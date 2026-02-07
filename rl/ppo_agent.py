import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.distributions import Normal
from typing import Tuple, List, Dict
import os


class ActorCritic(nn.Module):
    """
    Actor-Critic 네트워크
    Actor: 정책 네트워크 (행동 선택)
    Critic: 가치 네트워크 (상태 가치 평가)
    """
    
    def __init__(
        self,
        input_dim: int,
        action_dim: int,
        hidden_dims: List[int] = [256, 128, 64],
        activation: str = 'relu'
    ):
        """
        Args:
            input_dim: 입력 차원 (flatten된 observation 크기)
            action_dim: 행동 차원
            hidden_dims: 은닉층 크기 리스트
            activation: 활성화 함수
        """
        super(ActorCritic, self).__init__()
        
        self.input_dim = input_dim
        self.action_dim = action_dim
        
        # 활성화 함수 선택
        if activation == 'relu':
            self.activation = nn.ReLU()
        elif activation == 'tanh':
            self.activation = nn.Tanh()
        elif activation == 'elu':
            self.activation = nn.ELU()
        else:
            self.activation = nn.ReLU()
        
        # Shared feature extractor (LSTM for temporal data)
        self.feature_extractor = nn.Sequential(
            nn.Linear(input_dim, hidden_dims[0]),
            self.activation,
            nn.Dropout(0.2)
        )
        
        # Actor network (policy)
        actor_layers = []
        prev_dim = hidden_dims[0]
        for hidden_dim in hidden_dims[1:]:
            actor_layers.extend([
                nn.Linear(prev_dim, hidden_dim),
                self.activation,
                nn.Dropout(0.2)
            ])
            prev_dim = hidden_dim
        
        self.actor_backbone = nn.Sequential(*actor_layers)
        
        # Actor head: mean and log_std for Gaussian policy
        self.actor_mean = nn.Linear(prev_dim, action_dim)
        self.actor_log_std = nn.Linear(prev_dim, action_dim)
        
        # Critic network (value function)
        critic_layers = []
        prev_dim = hidden_dims[0]
        for hidden_dim in hidden_dims[1:]:
            critic_layers.extend([
                nn.Linear(prev_dim, hidden_dim),
                self.activation,
                nn.Dropout(0.2)
            ])
            prev_dim = hidden_dim
        
        self.critic_backbone = nn.Sequential(*critic_layers)
        self.critic_head = nn.Linear(prev_dim, 1)
        
        # Initialize weights
        self._initialize_weights()
    
    def _initialize_weights(self):
        """가중치 초기화"""
        for m in self.modules():
            if isinstance(m, nn.Linear):
                nn.init.orthogonal_(m.weight, gain=np.sqrt(2))
                nn.init.constant_(m.bias, 0.0)
    
    def forward(self, state: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """
        Forward pass
        
        Args:
            state: (batch_size, input_dim)
        
        Returns:
            action_mean: (batch_size, action_dim)
            action_log_std: (batch_size, action_dim)
            value: (batch_size, 1)
        """
        # Shared features
        features = self.feature_extractor(state)
        
        # Actor
        actor_features = self.actor_backbone(features)
        action_mean = torch.tanh(self.actor_mean(actor_features))  # [-1, 1] 범위로 제한
        action_log_std = self.actor_log_std(actor_features)
        action_log_std = torch.clamp(action_log_std, -20, 2)  # 안정성을 위해 clamp
        
        # Critic
        critic_features = self.critic_backbone(features)
        value = self.critic_head(critic_features)
        
        return action_mean, action_log_std, value
    
    def get_action(self, state: torch.Tensor, deterministic: bool = False) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """
        행동 샘플링
        
        Args:
            state: (batch_size, input_dim)
            deterministic: True면 평균값 사용, False면 샘플링
        
        Returns:
            action: (batch_size, action_dim)
            log_prob: (batch_size,)
            value: (batch_size, 1)
        """
        action_mean, action_log_std, value = self.forward(state)
        action_std = torch.exp(action_log_std)
        
        if deterministic:
            action = action_mean
            # 결정적 행동의 log_prob은 의미 없지만 형식상 계산
            dist = Normal(action_mean, action_std)
            log_prob = dist.log_prob(action).sum(dim=-1)
        else:
            dist = Normal(action_mean, action_std)
            action = dist.sample()
            log_prob = dist.log_prob(action).sum(dim=-1)
        
        # Action을 [-1, 1] 범위로 클램핑
        action = torch.clamp(action, -1.0, 1.0)
        
        return action, log_prob, value
    
    def evaluate_actions(self, state: torch.Tensor, action: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """
        행동 평가 (학습 시 사용)
        
        Args:
            state: (batch_size, input_dim)
            action: (batch_size, action_dim)
        
        Returns:
            log_prob: (batch_size,)
            entropy: (batch_size,)
            value: (batch_size, 1)
        """
        action_mean, action_log_std, value = self.forward(state)
        action_std = torch.exp(action_log_std)
        
        dist = Normal(action_mean, action_std)
        log_prob = dist.log_prob(action).sum(dim=-1)
        entropy = dist.entropy().sum(dim=-1)
        
        return log_prob, entropy, value


class PPOAgent:
    """
    Proximal Policy Optimization (PPO) 에이전트
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
        device: str = 'cuda' if torch.cuda.is_available() else 'cpu'
    ):
        """
        Args:
            state_dim: 상태 차원
            action_dim: 행동 차원
            hidden_dims: 은닉층 크기
            lr: 학습률
            gamma: 할인율
            gae_lambda: GAE lambda
            clip_epsilon: PPO clipping 파라미터
            value_loss_coef: 가치 손실 계수
            entropy_coef: 엔트로피 계수
            max_grad_norm: 그래디언트 클리핑 최대값
            device: 디바이스
        """
        self.device = device
        self.gamma = gamma
        self.gae_lambda = gae_lambda
        self.clip_epsilon = clip_epsilon
        self.value_loss_coef = value_loss_coef
        self.entropy_coef = entropy_coef
        self.max_grad_norm = max_grad_norm
        
        # Actor-Critic 네트워크
        self.actor_critic = ActorCritic(
            input_dim=state_dim,
            action_dim=action_dim,
            hidden_dims=hidden_dims
        ).to(device)
        
        # Optimizer
        self.optimizer = optim.Adam(self.actor_critic.parameters(), lr=lr, eps=1e-5)
        
        # 학습 통계
        self.training_stats = {
            'policy_loss': [],
            'value_loss': [],
            'entropy': [],
            'total_loss': [],
            'approx_kl': [],
            'clip_fraction': []
        }
    
    def select_action(self, state: np.ndarray, deterministic: bool = False) -> Tuple[np.ndarray, float, float]:
        """
        행동 선택
        
        Args:
            state: 상태 (observation)
            deterministic: 결정적 행동 선택 여부
        
        Returns:
            action: 선택된 행동
            log_prob: 로그 확률
            value: 상태 가치
        """
        # State를 flatten
        state_flat = state.flatten()
        state_tensor = torch.FloatTensor(state_flat).unsqueeze(0).to(self.device)
        
        with torch.no_grad():
            action, log_prob, value = self.actor_critic.get_action(state_tensor, deterministic)
        
        return action.cpu().numpy()[0], log_prob.cpu().item(), value.cpu().item()
    
    def compute_gae(
        self,
        rewards: List[float],
        values: List[float],
        dones: List[bool],
        next_value: float
    ) -> Tuple[List[float], List[float]]:
        """
        Generalized Advantage Estimation (GAE) 계산
        
        Args:
            rewards: 보상 리스트
            values: 가치 리스트
            dones: 종료 플래그 리스트
            next_value: 다음 상태 가치
        
        Returns:
            returns: 수익 리스트
            advantages: 이점 리스트
        """
        advantages = []
        gae = 0
        
        # 역순으로 계산
        for t in reversed(range(len(rewards))):
            if t == len(rewards) - 1:
                next_value_t = next_value
            else:
                next_value_t = values[t + 1]
            
            # TD error
            delta = rewards[t] + self.gamma * next_value_t * (1 - dones[t]) - values[t]
            
            # GAE
            gae = delta + self.gamma * self.gae_lambda * (1 - dones[t]) * gae
            advantages.insert(0, gae)
        
        # Returns = advantages + values
        returns = [adv + val for adv, val in zip(advantages, values)]
        
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
        PPO 업데이트
        
        Args:
            states: 상태 배열
            actions: 행동 배열
            old_log_probs: 이전 로그 확률
            returns: 수익 배열
            advantages: 이점 배열
            epochs: 에폭 수
            batch_size: 배치 크기
        
        Returns:
            학습 통계
        """
        # Numpy to Tensor
        states = torch.FloatTensor(states).to(self.device)
        actions = torch.FloatTensor(actions).to(self.device)
        old_log_probs = torch.FloatTensor(old_log_probs).to(self.device)
        returns = torch.FloatTensor(returns).to(self.device)
        advantages = torch.FloatTensor(advantages).to(self.device)
        
        # Normalize advantages
        advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)
        
        dataset_size = states.size(0)
        
        # 통계 저장
        epoch_policy_losses = []
        epoch_value_losses = []
        epoch_entropies = []
        epoch_total_losses = []
        epoch_approx_kls = []
        epoch_clip_fractions = []
        
        for epoch in range(epochs):
            # Mini-batch 학습
            indices = np.random.permutation(dataset_size)
            
            for start_idx in range(0, dataset_size, batch_size):
                end_idx = min(start_idx + batch_size, dataset_size)
                batch_indices = indices[start_idx:end_idx]
                
                batch_states = states[batch_indices]
                batch_actions = actions[batch_indices]
                batch_old_log_probs = old_log_probs[batch_indices]
                batch_returns = returns[batch_indices]
                batch_advantages = advantages[batch_indices]
                
                # Evaluate actions
                log_probs, entropy, values = self.actor_critic.evaluate_actions(
                    batch_states, batch_actions
                )
                
                # Policy loss (PPO clipping)
                ratio = torch.exp(log_probs - batch_old_log_probs)
                surr1 = ratio * batch_advantages
                surr2 = torch.clamp(ratio, 1.0 - self.clip_epsilon, 1.0 + self.clip_epsilon) * batch_advantages
                policy_loss = -torch.min(surr1, surr2).mean()
                
                # Value loss
                value_loss = nn.MSELoss()(values.squeeze(), batch_returns)
                
                # Entropy bonus
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
                
                # 통계
                with torch.no_grad():
                    approx_kl = (batch_old_log_probs - log_probs).mean().item()
                    clip_fraction = ((ratio - 1.0).abs() > self.clip_epsilon).float().mean().item()
                
                epoch_policy_losses.append(policy_loss.item())
                epoch_value_losses.append(value_loss.item())
                epoch_entropies.append(entropy.mean().item())
                epoch_total_losses.append(loss.item())
                epoch_approx_kls.append(approx_kl)
                epoch_clip_fractions.append(clip_fraction)
        
        # 평균 통계
        stats = {
            'policy_loss': np.mean(epoch_policy_losses),
            'value_loss': np.mean(epoch_value_losses),
            'entropy': np.mean(epoch_entropies),
            'total_loss': np.mean(epoch_total_losses),
            'approx_kl': np.mean(epoch_approx_kls),
            'clip_fraction': np.mean(epoch_clip_fractions)
        }
        
        # 통계 저장
        for key, value in stats.items():
            self.training_stats[key].append(value)
        
        return stats
    
    def save(self, filepath: str):
        """모델 저장"""
        os.makedirs(os.path.dirname(filepath), exist_ok=True)
        torch.save({
            'actor_critic_state_dict': self.actor_critic.state_dict(),
            'optimizer_state_dict': self.optimizer.state_dict(),
            'training_stats': self.training_stats
        }, filepath)
        print(f"Model saved to {filepath}")
    
    def load(self, filepath: str):
        """모델 로드"""
        checkpoint = torch.load(filepath, map_location=self.device)
        self.actor_critic.load_state_dict(checkpoint['actor_critic_state_dict'])
        self.optimizer.load_state_dict(checkpoint['optimizer_state_dict'])
        self.training_stats = checkpoint['training_stats']
        print(f"Model loaded from {filepath}")


class RecurrentActorCritic(nn.Module):
    """
    LSTM 기반 Recurrent Actor-Critic 네트워크
    시계열 데이터의 temporal dependency를 더 잘 포착
    """
    
    def __init__(
        self,
        input_dim: int,
        action_dim: int,
        hidden_dim: int = 128,
        num_layers: int = 2
    ):
        super(RecurrentActorCritic, self).__init__()
        
        self.hidden_dim = hidden_dim
        self.num_layers = num_layers
        
        # LSTM for temporal features
        self.lstm = nn.LSTM(
            input_size=input_dim,
            hidden_size=hidden_dim,
            num_layers=num_layers,
            batch_first=True,
            dropout=0.2 if num_layers > 1 else 0
        )
        
        # Actor head
        self.actor_mean = nn.Linear(hidden_dim, action_dim)
        self.actor_log_std = nn.Linear(hidden_dim, action_dim)
        
        # Critic head
        self.critic = nn.Linear(hidden_dim, 1)
        
        self._initialize_weights()
    
    def _initialize_weights(self):
        """가중치 초기화"""
        for name, param in self.lstm.named_parameters():
            if 'weight' in name:
                nn.init.orthogonal_(param)
            elif 'bias' in name:
                nn.init.constant_(param, 0.0)
        
        nn.init.orthogonal_(self.actor_mean.weight, gain=0.01)
        nn.init.orthogonal_(self.actor_log_std.weight, gain=0.01)
        nn.init.orthogonal_(self.critic.weight, gain=1.0)
    
    def forward(self, state: torch.Tensor, hidden_state=None):
        """
        Forward pass with LSTM
        
        Args:
            state: (batch_size, seq_len, input_dim) or (batch_size, input_dim)
            hidden_state: LSTM hidden state
        
        Returns:
            action_mean, action_log_std, value, new_hidden_state
        """
        # state가 2D면 3D로 변환
        if state.dim() == 2:
            state = state.unsqueeze(1)
        
        # LSTM forward
        lstm_out, new_hidden_state = self.lstm(state, hidden_state)
        
        # 마지막 타임스텝의 출력 사용
        features = lstm_out[:, -1, :]
        
        # Actor
        action_mean = torch.tanh(self.actor_mean(features))
        action_log_std = self.actor_log_std(features)
        action_log_std = torch.clamp(action_log_std, -20, 2)
        
        # Critic
        value = self.critic(features)
        
        return action_mean, action_log_std, value, new_hidden_state