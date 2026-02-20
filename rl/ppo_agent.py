# rl/ppo_agent.py (V3 - 탐험 조정)
"""
PPO 에이전트 V3 - 더 집중된 학습

변경사항:
1. 엔트로피 감소 (0.1 → 0.02)
2. Epsilon 감소 (0.5 → 0.3)
3. 노이즈 감소 (1.0 → 0.5)
"""

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from typing import Tuple, List, Dict, Optional
import os

from .networks import ActorCritic, RecurrentActorCritic


class PPOAgent:
    """PPO 에이전트 V3"""
    
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
        entropy_coef: float = 0.02,  # 🔥 0.1 → 0.02
        max_grad_norm: float = 0.5,
        use_recurrent: bool = False,
        device: str = 'cuda' if torch.cuda.is_available() else 'cpu'
    ):
        self.device = device
        self.gamma = gamma
        self.gae_lambda = gae_lambda
        self.clip_epsilon = clip_epsilon
        self.value_loss_coef = value_loss_coef
        self.entropy_coef = entropy_coef
        self.max_grad_norm = max_grad_norm
        self.use_recurrent = use_recurrent
        
        # 🔥 V3: 더 보수적인 탐험
        self.action_noise_std = 0.5        # 1.0 → 0.5
        self.noise_decay = 0.998           # 0.999 → 0.998 (더 빠른 감소)
        self.min_noise_std = 0.1           # 0.2 → 0.1
        
        # Epsilon-greedy (더 낮게)
        self.epsilon = 0.3                 # 0.5 → 0.3
        self.epsilon_decay = 0.993         # 0.995 → 0.993 (더 빠른 감소)
        self.epsilon_min = 0.01
        
        self.total_steps = 0
        self.update_count = 0
        self.episode_count = 0
        
        # Actor-Critic 네트워크
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
        
        self._initialize_weights()
        
        self.optimizer = optim.Adam(self.actor_critic.parameters(), lr=lr, eps=1e-5)
        self.scheduler = optim.lr_scheduler.StepLR(self.optimizer, step_size=1000, gamma=0.95)
        
        self.training_stats = {
            'policy_loss': [],
            'value_loss': [],
            'entropy': [],
            'total_loss': [],
            'approx_kl': [],
            'clip_fraction': [],
            'explained_variance': []
        }
        
        self.episode_stats = {
            'rewards': [],
            'lengths': [],
            'profits': []
        }
    
    def _initialize_weights(self):
        for module in self.actor_critic.modules():
            if isinstance(module, nn.Linear):
                nn.init.orthogonal_(module.weight, gain=np.sqrt(2))
                if module.bias is not None:
                    nn.init.constant_(module.bias, 0)
    
    def select_action(
        self,
        state: np.ndarray,
        deterministic: bool = False
    ) -> Tuple[np.ndarray, float, float]:
        """행동 선택"""
        if state.ndim > 1:
            state_flat = state.flatten()
        else:
            state_flat = state
        
        state_tensor = torch.FloatTensor(state_flat).unsqueeze(0).to(self.device)
        
        # Epsilon-greedy
        if not deterministic and np.random.rand() < self.epsilon:
            action_np = np.random.uniform(-1.0, 1.0, size=(1,))
            
            with torch.no_grad():
                if self.use_recurrent:
                    _, log_prob, value, _ = self.actor_critic.get_action(
                        state_tensor,
                        deterministic=False
                    )
                else:
                    _, log_prob, value = self.actor_critic.get_action(
                        state_tensor,
                        deterministic=False
                    )
            
            return action_np, log_prob.cpu().item(), value.cpu().item()
        
        # 정책 기반
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
        
        action_np = action.cpu().numpy()[0]
        
        # Gaussian Noise
        if not deterministic and self.action_noise_std > self.min_noise_std:
            noise = np.random.normal(0, self.action_noise_std, size=action_np.shape)
            action_np = np.clip(action_np + noise, -1.0, 1.0)
        
        self.total_steps += 1
        
        return action_np, log_prob.cpu().item(), value.cpu().item()
    
    def decay_exploration(self):
        """탐험 파라미터 감소"""
        self.action_noise_std = max(
            self.min_noise_std,
            self.action_noise_std * self.noise_decay
        )
        
        self.epsilon = max(
            self.epsilon_min,
            self.epsilon * self.epsilon_decay
        )
    
    def compute_gae(
        self,
        rewards: List[float],
        values: List[float],
        dones: List[bool],
        next_value: float
    ) -> Tuple[np.ndarray, np.ndarray]:
        """GAE 계산"""
        advantages = []
        gae = 0
        
        for t in reversed(range(len(rewards))):
            if t == len(rewards) - 1:
                next_value_t = next_value
            else:
                next_value_t = values[t + 1]
            
            delta = rewards[t] + self.gamma * next_value_t * (1 - dones[t]) - values[t]
            gae = delta + self.gamma * self.gae_lambda * (1 - dones[t]) * gae
            advantages.insert(0, gae)
        
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
        """PPO 업데이트"""
        
        states = torch.FloatTensor(states).to(self.device)
        actions = torch.FloatTensor(actions).to(self.device)
        old_log_probs = torch.FloatTensor(old_log_probs).to(self.device)
        returns = torch.FloatTensor(returns).to(self.device)
        advantages = torch.FloatTensor(advantages).to(self.device)
        
        advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)
        
        dataset_size = states.size(0)
        
        epoch_policy_losses = []
        epoch_value_losses = []
        epoch_entropies = []
        epoch_total_losses = []
        epoch_approx_kls = []
        epoch_clip_fractions = []
        
        for epoch in range(epochs):
            indices = np.random.permutation(dataset_size)
            
            for start_idx in range(0, dataset_size, batch_size):
                end_idx = min(start_idx + batch_size, dataset_size)
                batch_indices = indices[start_idx:end_idx]
                
                batch_states = states[batch_indices]
                batch_actions = actions[batch_indices]
                batch_old_log_probs = old_log_probs[batch_indices]
                batch_returns = returns[batch_indices]
                batch_advantages = advantages[batch_indices]
                
                log_probs, entropy, values = self.actor_critic.evaluate_actions(
                    batch_states, batch_actions
                )
                
                ratio = torch.exp(log_probs - batch_old_log_probs)
                surr1 = ratio * batch_advantages
                surr2 = torch.clamp(
                    ratio,
                    1.0 - self.clip_epsilon,
                    1.0 + self.clip_epsilon
                ) * batch_advantages
                policy_loss = -torch.min(surr1, surr2).mean()
                
                value_loss = nn.SmoothL1Loss()(values.squeeze(), batch_returns)
                entropy_loss = -entropy.mean()
                
                loss = (
                    policy_loss +
                    self.value_loss_coef * value_loss +
                    self.entropy_coef * entropy_loss
                )
                
                self.optimizer.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(self.actor_critic.parameters(), self.max_grad_norm)
                self.optimizer.step()
                
                with torch.no_grad():
                    approx_kl = (batch_old_log_probs - log_probs).mean().item()
                    clip_fraction = ((ratio - 1.0).abs() > self.clip_epsilon).float().mean().item()
                
                epoch_policy_losses.append(policy_loss.item())
                epoch_value_losses.append(value_loss.item())
                epoch_entropies.append(entropy.mean().item())
                epoch_total_losses.append(loss.item())
                epoch_approx_kls.append(approx_kl)
                epoch_clip_fractions.append(clip_fraction)
            
            mean_kl = np.mean(epoch_approx_kls[-len(indices)//batch_size:])
            if mean_kl > 0.03:
                break
        
        self.scheduler.step()
        self.update_count += 1
        
        with torch.no_grad():
            _, _, all_values = self.actor_critic.evaluate_actions(states, actions)
            all_values = all_values.squeeze()
            explained_var = 1 - torch.var(returns - all_values) / (torch.var(returns) + 1e-8)
            explained_var = explained_var.item()
        
        stats = {
            'policy_loss': np.mean(epoch_policy_losses),
            'value_loss': np.mean(epoch_value_losses),
            'entropy': np.mean(epoch_entropies),
            'total_loss': np.mean(epoch_total_losses),
            'approx_kl': np.mean(epoch_approx_kls),
            'clip_fraction': np.mean(epoch_clip_fractions),
            'explained_variance': explained_var
        }
        
        for key, value in stats.items():
            self.training_stats[key].append(value)
        
        return stats
    
    def save(self, filepath: str):
        os.makedirs(os.path.dirname(filepath), exist_ok=True)
        torch.save({
            'actor_critic_state_dict': self.actor_critic.state_dict(),
            'optimizer_state_dict': self.optimizer.state_dict(),
            'scheduler_state_dict': self.scheduler.state_dict(),
            'training_stats': self.training_stats,
            'episode_stats': self.episode_stats,
            'total_steps': self.total_steps,
            'update_count': self.update_count,
            'action_noise_std': self.action_noise_std,
            'epsilon': self.epsilon,
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
    
    def load(self, filepath: str):
        checkpoint = torch.load(filepath, map_location=self.device)
        self.actor_critic.load_state_dict(checkpoint['actor_critic_state_dict'])
        self.optimizer.load_state_dict(checkpoint['optimizer_state_dict'])
        
        if 'scheduler_state_dict' in checkpoint:
            self.scheduler.load_state_dict(checkpoint['scheduler_state_dict'])
        if 'training_stats' in checkpoint:
            self.training_stats = checkpoint['training_stats']
        if 'episode_stats' in checkpoint:
            self.episode_stats = checkpoint['episode_stats']
        if 'total_steps' in checkpoint:
            self.total_steps = checkpoint['total_steps']
        if 'update_count' in checkpoint:
            self.update_count = checkpoint['update_count']
        if 'action_noise_std' in checkpoint:
            self.action_noise_std = checkpoint['action_noise_std']
        if 'epsilon' in checkpoint:
            self.epsilon = checkpoint['epsilon']
    
    def get_training_stats(self) -> Dict[str, List[float]]:
        return self.training_stats
    
    def get_episode_stats(self) -> Dict[str, List[float]]:
        return self.episode_stats
    
    def add_episode_stats(self, reward: float, length: int, profit: float):
        self.episode_stats['rewards'].append(reward)
        self.episode_stats['lengths'].append(length)
        self.episode_stats['profits'].append(profit)
        self.episode_count += 1