# rl/ppo_agent.py
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from typing import Tuple, List, Dict, Optional
import os

from .networks import ActorCritic, RecurrentActorCritic


class PPOAgent:
    """
    Proximal Policy Optimization (PPO) 에이전트
    
    PPO는 다음과 같은 특징을 가진 강화학습 알고리즘입니다:
    - On-policy: 현재 정책으로 수집한 데이터로 학습
    - Clipped surrogate objective: 정책 업데이트를 안정적으로 제한
    - Actor-Critic: 정책과 가치 함수를 동시에 학습
    """
    
    def __init__(
        self,
        state_dim: int,
        action_dim: int,
        hidden_dims: List[int] = [256, 128, 64],
        lr: float = 3e-4,
        gamma: float = 0.99,
        gae_lambda: float = 0.95,
        clip_epsilon: float = 0.2,
        value_loss_coef: float = 0.5,
        entropy_coef: float = 0.01,
        max_grad_norm: float = 0.5,
        use_recurrent: bool = False,
        device: str = 'cuda' if torch.cuda.is_available() else 'cpu'
    ):
        """
        Args:
            state_dim: 상태 차원 (flatten된 observation 크기)
            action_dim: 행동 차원
            hidden_dims: 은닉층 크기 리스트
            lr: 학습률 (learning rate)
            gamma: 할인율 (discount factor)
            gae_lambda: GAE lambda 파라미터
            clip_epsilon: PPO clipping 파라미터 (epsilon)
            value_loss_coef: 가치 손실 계수
            entropy_coef: 엔트로피 보너스 계수
            max_grad_norm: 그래디언트 클리핑 최대 norm
            use_recurrent: LSTM 네트워크 사용 여부
            device: 학습 디바이스 ('cuda' or 'cpu')
        """
        self.device = device
        self.gamma = gamma
        self.gae_lambda = gae_lambda
        self.clip_epsilon = clip_epsilon
        self.value_loss_coef = value_loss_coef
        self.entropy_coef = entropy_coef
        self.max_grad_norm = max_grad_norm
        self.use_recurrent = use_recurrent
        
        # Actor-Critic 네트워크 선택
        if use_recurrent:
            self.actor_critic = RecurrentActorCritic(
                input_dim=state_dim,
                action_dim=action_dim,
                hidden_dim=hidden_dims[0] if hidden_dims else 128,
                num_layers=2
            ).to(device)
        else:
            self.actor_critic = ActorCritic(
                input_dim=state_dim,
                action_dim=action_dim,
                hidden_dims=hidden_dims
            ).to(device)
        
        # Optimizer
        self.optimizer = optim.Adam(self.actor_critic.parameters(), lr=lr, eps=1e-5)
        
        # 학습 스케줄러 (optional)
        self.scheduler = optim.lr_scheduler.StepLR(self.optimizer, step_size=1000, gamma=0.95)
        
        # 학습 통계
        self.training_stats = {
            'policy_loss': [],
            'value_loss': [],
            'entropy': [],
            'total_loss': [],
            'approx_kl': [],
            'clip_fraction': [],
            'explained_variance': []
        }
        
        # 에피소드 통계
        self.episode_stats = {
            'rewards': [],
            'lengths': [],
            'profits': []
        }
    
    def select_action(
        self,
        state: np.ndarray,
        deterministic: bool = False
    ) -> Tuple[np.ndarray, float, float]:
        """
        행동 선택
        
        Args:
            state: 상태 (observation) - shape: (window_size, n_features) or flattened
            deterministic: True면 평균값 사용, False면 확률적 샘플링
        
        Returns:
            action: 선택된 행동 - shape: (action_dim,)
            log_prob: 행동의 로그 확률
            value: 상태의 가치 추정값
        """
        # State를 flatten (이미 flatten되어 있으면 그대로)
        if state.ndim > 1:
            state_flat = state.flatten()
        else:
            state_flat = state
        
        state_tensor = torch.FloatTensor(state_flat).unsqueeze(0).to(self.device)
        
        with torch.no_grad():
            if self.use_recurrent:
                action, log_prob, value, _ = self.actor_critic.get_action(
                    state_tensor,
                    deterministic=deterministic
                )
            else:
                action, log_prob, value = self.actor_critic.get_action(
                    state_tensor,
                    deterministic=deterministic
                )
        
        return action.cpu().numpy()[0], log_prob.cpu().item(), value.cpu().item()
    
    def compute_gae(
        self,
        rewards: List[float],
        values: List[float],
        dones: List[bool],
        next_value: float
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Generalized Advantage Estimation (GAE) 계산
        
        GAE는 advantage 함수의 분산을 줄이기 위한 기법입니다.
        A_t = δ_t + (γλ)δ_{t+1} + (γλ)^2 δ_{t+2} + ...
        where δ_t = r_t + γV(s_{t+1}) - V(s_t)
        
        Args:
            rewards: 보상 리스트
            values: 가치 추정값 리스트
            dones: 에피소드 종료 플래그 리스트
            next_value: 다음 상태의 가치
        
        Returns:
            returns: 수익 (return) 배열 - shape: (T,)
            advantages: 이점 (advantage) 배열 - shape: (T,)
        """
        advantages = []
        gae = 0
        
        # 역순으로 계산 (t = T-1, T-2, ..., 0)
        for t in reversed(range(len(rewards))):
            if t == len(rewards) - 1:
                next_value_t = next_value
            else:
                next_value_t = values[t + 1]
            
            # TD error: δ_t = r_t + γV(s_{t+1}) - V(s_t)
            delta = rewards[t] + self.gamma * next_value_t * (1 - dones[t]) - values[t]
            
            # GAE: A_t = δ_t + (γλ)(1-done)A_{t+1}
            gae = delta + self.gamma * self.gae_lambda * (1 - dones[t]) * gae
            advantages.insert(0, gae)
        
        # Returns = advantages + values
        advantages = np.array(advantages, dtype=np.float32)
        values_array = np.array(values, dtype=np.float32)
        returns = advantages + values_array
        
        return returns, advantages
    
    def update(
        self,
        states: np.ndarray,
        actions: np.ndarray,
        old_log_probs: np.ndarray,
        returns: np.ndarray,
        advantages: np.ndarray,
        epochs: int = 10,
        batch_size: int = 64
    ) -> Dict[str, float]:
        """
        PPO 업데이트 (여러 에폭 동안 미니배치 학습)
        
        Args:
            states: 상태 배열 - shape: (T, state_dim)
            actions: 행동 배열 - shape: (T, action_dim)
            old_log_probs: 이전 정책의 로그 확률 - shape: (T,)
            returns: 수익 배열 - shape: (T,)
            advantages: 이점 배열 - shape: (T,)
            epochs: 학습 에폭 수
            batch_size: 미니배치 크기
        
        Returns:
            학습 통계 딕셔너리
        """
        # Numpy to Tensor
        states = torch.FloatTensor(states).to(self.device)
        actions = torch.FloatTensor(actions).to(self.device)
        old_log_probs = torch.FloatTensor(old_log_probs).to(self.device)
        returns = torch.FloatTensor(returns).to(self.device)
        advantages = torch.FloatTensor(advantages).to(self.device)
        
        # Normalize advantages (중요!)
        advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)
        
        dataset_size = states.size(0)
        
        # 통계 저장용 리스트
        epoch_policy_losses = []
        epoch_value_losses = []
        epoch_entropies = []
        epoch_total_losses = []
        epoch_approx_kls = []
        epoch_clip_fractions = []
        
        # 여러 에폭 동안 학습
        for epoch in range(epochs):
            # 데이터 셔플
            indices = np.random.permutation(dataset_size)
            
            # 미니배치 학습
            for start_idx in range(0, dataset_size, batch_size):
                end_idx = min(start_idx + batch_size, dataset_size)
                batch_indices = indices[start_idx:end_idx]
                
                batch_states = states[batch_indices]
                batch_actions = actions[batch_indices]
                batch_old_log_probs = old_log_probs[batch_indices]
                batch_returns = returns[batch_indices]
                batch_advantages = advantages[batch_indices]
                
                # 현재 정책으로 행동 평가
                log_probs, entropy, values = self.actor_critic.evaluate_actions(
                    batch_states, batch_actions
                )
                
                # Policy loss (PPO clipped objective)
                # L^CLIP = E[min(r_t(θ)Â_t, clip(r_t(θ), 1-ε, 1+ε)Â_t)]
                ratio = torch.exp(log_probs - batch_old_log_probs)
                surr1 = ratio * batch_advantages
                surr2 = torch.clamp(
                    ratio,
                    1.0 - self.clip_epsilon,
                    1.0 + self.clip_epsilon
                ) * batch_advantages
                policy_loss = -torch.min(surr1, surr2).mean()
                
                # Value loss (클리핑 적용)
                mse_loss = nn.MSELoss()(values.squeeze(), batch_returns)
                value_loss = torch.clamp(mse_loss, min=0.0, max=100.0)
                
                # Entropy bonus (탐험 장려)
                entropy_loss = -entropy.mean()
                
                # Total loss
                loss = (
                    policy_loss +
                    self.value_loss_coef * value_loss +
                    self.entropy_coef * entropy_loss
                )
                
                # Backward pass
                self.optimizer.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(self.actor_critic.parameters(), self.max_grad_norm)
                self.optimizer.step()
                
                # 통계 계산
                with torch.no_grad():
                    # KL divergence 근사
                    approx_kl = (batch_old_log_probs - log_probs).mean().item()
                    
                    # Clipping fraction (얼마나 많은 샘플이 clipping되었는지)
                    clip_fraction = ((ratio - 1.0).abs() > self.clip_epsilon).float().mean().item()
                
                epoch_policy_losses.append(policy_loss.item())
                epoch_value_losses.append(value_loss.item())
                epoch_entropies.append(entropy.mean().item())
                epoch_total_losses.append(loss.item())
                epoch_approx_kls.append(approx_kl)
                epoch_clip_fractions.append(clip_fraction)
        
        # Learning rate scheduler step
        self.scheduler.step()
        
        # Explained variance 계산
        with torch.no_grad():
            _, _, all_values = self.actor_critic.evaluate_actions(states, actions)
            all_values = all_values.squeeze()
            explained_var = 1 - torch.var(returns - all_values) / (torch.var(returns) + 1e-8)
            explained_var = explained_var.item()
        
        # 평균 통계
        stats = {
            'policy_loss': np.mean(epoch_policy_losses),
            'value_loss': np.mean(epoch_value_losses),
            'entropy': np.mean(epoch_entropies),
            'total_loss': np.mean(epoch_total_losses),
            'approx_kl': np.mean(epoch_approx_kls),
            'clip_fraction': np.mean(epoch_clip_fractions),
            'explained_variance': explained_var
        }
        
        # 통계 저장
        for key, value in stats.items():
            self.training_stats[key].append(value)
        
        return stats
    
    def save(self, filepath: str):
        """
        모델 저장
        
        Args:
            filepath: 저장 경로
        """
        os.makedirs(os.path.dirname(filepath), exist_ok=True)
        torch.save({
            'actor_critic_state_dict': self.actor_critic.state_dict(),
            'optimizer_state_dict': self.optimizer.state_dict(),
            'scheduler_state_dict': self.scheduler.state_dict(),
            'training_stats': self.training_stats,
            'episode_stats': self.episode_stats,
            'config': {
                'gamma': self.gamma,
                'gae_lambda': self.gae_lambda,
                'clip_epsilon': self.clip_epsilon,
                'value_loss_coef': self.value_loss_coef,
                'entropy_coef': self.entropy_coef,
                'max_grad_norm': self.max_grad_norm,
                'use_recurrent': self.use_recurrent
            }
        }, filepath)
        print(f"✅ Model saved to {filepath}")
    
    def load(self, filepath: str):
        """
        모델 로드
        
        Args:
            filepath: 로드 경로
        """
        checkpoint = torch.load(filepath, map_location=self.device)
        self.actor_critic.load_state_dict(checkpoint['actor_critic_state_dict'])
        self.optimizer.load_state_dict(checkpoint['optimizer_state_dict'])
        
        if 'scheduler_state_dict' in checkpoint:
            self.scheduler.load_state_dict(checkpoint['scheduler_state_dict'])
        
        if 'training_stats' in checkpoint:
            self.training_stats = checkpoint['training_stats']
        
        if 'episode_stats' in checkpoint:
            self.episode_stats = checkpoint['episode_stats']
        
        print(f"✅ Model loaded from {filepath}")
    
    def get_training_stats(self) -> Dict[str, List[float]]:
        """학습 통계 반환"""
        return self.training_stats
    
    def get_episode_stats(self) -> Dict[str, List[float]]:
        """에피소드 통계 반환"""
        return self.episode_stats
    
    def add_episode_stats(self, reward: float, length: int, profit: float):
        """에피소드 통계 추가"""
        self.episode_stats['rewards'].append(reward)
        self.episode_stats['lengths'].append(length)
        self.episode_stats['profits'].append(profit)