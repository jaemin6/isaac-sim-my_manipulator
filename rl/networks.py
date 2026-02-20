# rl/networks.py (FIXED - Action 고정 문제 해결)
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.distributions import Normal
from typing import Tuple


class ActorCritic(nn.Module):
    """
    Actor-Critic 네트워크 (개선 버전)
    
    문제: Action이 1.0으로 고정됨
    원인: 
    1. Actor의 std가 너무 작음 (탐험 없음)
    2. 네트워크 초기화가 잘못됨
    3. Tanh 활성화가 1.0으로 saturation
    
    해결:
    1. Log std를 학습 가능한 파라미터로 변경
    2. 초기화 개선 (Xavier → Orthogonal)
    3. Std 최소값 강제
    """
    
    def __init__(
        self,
        input_dim: int,
        action_dim: int,
        hidden_dims: list = [256, 128, 64]
    ):
        super(ActorCritic, self).__init__()
        
        self.input_dim = input_dim
        self.action_dim = action_dim
        
        # ========================================
        # Shared Feature Extractor
        # ========================================
        layers = []
        prev_dim = input_dim
        
        for hidden_dim in hidden_dims:
            layers.append(nn.Linear(prev_dim, hidden_dim))
            layers.append(nn.LayerNorm(hidden_dim))  # 🔥 BatchNorm → LayerNorm
            layers.append(nn.ReLU())
            layers.append(nn.Dropout(0.1))  # 🆕 Dropout 추가
            prev_dim = hidden_dim
        
        self.feature_extractor = nn.Sequential(*layers)
        
        # ========================================
        # Actor Head (Policy)
        # ========================================
        self.actor_mean = nn.Sequential(
            nn.Linear(prev_dim, 64),
            nn.ReLU(),
            nn.Linear(64, action_dim),
            nn.Tanh()  # [-1, 1] 범위
        )
        
        # 🔥 Log std를 학습 가능한 파라미터로 (고정값 대신)
        self.actor_log_std = nn.Parameter(
            torch.zeros(1, action_dim)  # 초기값 0 (std=1.0)
        )
        
        # ========================================
        # Critic Head (Value)
        # ========================================
        self.critic = nn.Sequential(
            nn.Linear(prev_dim, 64),
            nn.ReLU(),
            nn.Linear(64, 1)
        )
        
        # 🔥 네트워크 초기화
        self._initialize_weights()
    
    def _initialize_weights(self):
        """
        개선된 가중치 초기화
        
        Orthogonal 초기화: RL에서 가장 효과적
        """
        for module in self.modules():
            if isinstance(module, nn.Linear):
                # Orthogonal initialization
                nn.init.orthogonal_(module.weight, gain=0.01)  # 🔥 작은 gain
                if module.bias is not None:
                    nn.init.constant_(module.bias, 0)
        
        # Actor mean의 마지막 레이어는 더 작게
        nn.init.orthogonal_(self.actor_mean[-2].weight, gain=0.01)
        nn.init.constant_(self.actor_mean[-2].bias, 0)
    
    def forward(self, state: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor]:
        """
        Forward pass
        
        Returns:
            action_mean: 행동 평균
            value: 상태 가치
        """
        features = self.feature_extractor(state)
        action_mean = self.actor_mean(features)
        value = self.critic(features)
        
        return action_mean, value
    
    def get_action(
        self,
        state: torch.Tensor,
        deterministic: bool = False
    ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """
        행동 샘플링
        
        Args:
            state: 상태
            deterministic: True면 평균값 사용
        
        Returns:
            action: 샘플링된 행동
            log_prob: 로그 확률
            value: 상태 가치
        """
        action_mean, value = self.forward(state)
        
        # 🔥 Std 계산 (최소값 강제)
        action_log_std = self.actor_log_std.expand_as(action_mean)
        action_std = torch.exp(action_log_std)
        
        # 🔥 최소 std 강제 (0.2 이상)
        action_std = torch.clamp(action_std, min=0.2, max=2.0)
        
        # 정규 분포 생성
        dist = Normal(action_mean, action_std)
        
        if deterministic:
            action = action_mean
        else:
            action = dist.sample()
        
        # 행동 클리핑 [-1, 1]
        action = torch.clamp(action, -1.0, 1.0)
        
        # 로그 확률 계산
        log_prob = dist.log_prob(action).sum(dim=-1)
        
        return action, log_prob, value
    
    def evaluate_actions(
        self,
        state: torch.Tensor,
        action: torch.Tensor
    ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """
        행동 평가 (학습용)
        
        Returns:
            log_prob: 행동의 로그 확률
            entropy: 정책의 엔트로피
            value: 상태 가치
        """
        action_mean, value = self.forward(state)
        
        # Std 계산
        action_log_std = self.actor_log_std.expand_as(action_mean)
        action_std = torch.exp(action_log_std)
        action_std = torch.clamp(action_std, min=0.2, max=2.0)
        
        # 분포 생성
        dist = Normal(action_mean, action_std)
        
        # 로그 확률
        log_prob = dist.log_prob(action).sum(dim=-1)
        
        # 엔트로피 (탐험 지표)
        entropy = dist.entropy().sum(dim=-1)
        
        return log_prob, entropy, value


class RecurrentActorCritic(nn.Module):
    """
    LSTM 기반 Actor-Critic (시계열 데이터용)
    """
    
    def __init__(
        self,
        input_dim: int,
        action_dim: int,
        hidden_dim: int = 128,
        num_layers: int = 2
    ):
        super(RecurrentActorCritic, self).__init__()
        
        self.input_dim = input_dim
        self.action_dim = action_dim
        self.hidden_dim = hidden_dim
        self.num_layers = num_layers
        
        # LSTM
        self.lstm = nn.LSTM(
            input_size=input_dim,
            hidden_size=hidden_dim,
            num_layers=num_layers,
            batch_first=True
        )
        
        # Actor
        self.actor_mean = nn.Sequential(
            nn.Linear(hidden_dim, 64),
            nn.ReLU(),
            nn.Linear(64, action_dim),
            nn.Tanh()
        )
        
        self.actor_log_std = nn.Parameter(torch.zeros(1, action_dim))
        
        # Critic
        self.critic = nn.Sequential(
            nn.Linear(hidden_dim, 64),
            nn.ReLU(),
            nn.Linear(64, 1)
        )
        
        self._initialize_weights()
    
    def _initialize_weights(self):
        """가중치 초기화"""
        for name, param in self.named_parameters():
            if 'weight' in name:
                if 'lstm' in name:
                    nn.init.orthogonal_(param, gain=1.0)
                else:
                    nn.init.orthogonal_(param, gain=0.01)
            elif 'bias' in name:
                nn.init.constant_(param, 0)
    
    def forward(
        self,
        state: torch.Tensor,
        hidden: Tuple[torch.Tensor, torch.Tensor] = None
    ) -> Tuple[torch.Tensor, torch.Tensor, Tuple[torch.Tensor, torch.Tensor]]:
        """
        Forward pass
        
        Returns:
            action_mean, value, (h, c)
        """
        # LSTM
        if hidden is None:
            lstm_out, hidden = self.lstm(state)
        else:
            lstm_out, hidden = self.lstm(state, hidden)
        
        # 마지막 타임스텝의 출력 사용
        features = lstm_out[:, -1, :]
        
        action_mean = self.actor_mean(features)
        value = self.critic(features)
        
        return action_mean, value, hidden
    
    def get_action(
        self,
        state: torch.Tensor,
        hidden: Tuple[torch.Tensor, torch.Tensor] = None,
        deterministic: bool = False
    ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor, Tuple[torch.Tensor, torch.Tensor]]:
        """
        행동 샘플링
        
        Returns:
            action, log_prob, value, hidden
        """
        action_mean, value, hidden = self.forward(state, hidden)
        
        action_log_std = self.actor_log_std.expand_as(action_mean)
        action_std = torch.exp(action_log_std)
        action_std = torch.clamp(action_std, min=0.2, max=2.0)
        
        dist = Normal(action_mean, action_std)
        
        if deterministic:
            action = action_mean
        else:
            action = dist.sample()
        
        action = torch.clamp(action, -1.0, 1.0)
        log_prob = dist.log_prob(action).sum(dim=-1)
        
        return action, log_prob, value, hidden
    
    def evaluate_actions(
        self,
        state: torch.Tensor,
        action: torch.Tensor
    ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """행동 평가"""
        action_mean, value, _ = self.forward(state)
        
        action_log_std = self.actor_log_std.expand_as(action_mean)
        action_std = torch.exp(action_log_std)
        action_std = torch.clamp(action_std, min=0.2, max=2.0)
        
        dist = Normal(action_mean, action_std)
        
        log_prob = dist.log_prob(action).sum(dim=-1)
        entropy = dist.entropy().sum(dim=-1)
        
        return log_prob, entropy, value