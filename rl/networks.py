# rl/networks.py
import numpy as np
import torch
import torch.nn as nn
from typing import List, Tuple


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
            activation: 활성화 함수 ('relu', 'tanh', 'elu')
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
        
        # Shared feature extractor
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
        """가중치 초기화 - Orthogonal initialization"""
        for m in self.modules():
            if isinstance(m, nn.Linear):
                nn.init.orthogonal_(m.weight, gain=np.sqrt(2))
                nn.init.constant_(m.bias, 0.0)
        
        # Actor mean head는 매우 작은 값으로 초기화 (초반 랜덤 행동 방지)
        nn.init.normal_(self.actor_mean.weight, mean=0.0, std=0.001)  # orthogonal → normal, std 매우 작게!
        nn.init.constant_(self.actor_mean.bias, 0.0)
    
    def forward(self, state: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """
        Forward pass
        
        Args:
            state: (batch_size, input_dim)
        
        Returns:
            action_mean: (batch_size, action_dim) - 행동의 평균
            action_log_std: (batch_size, action_dim) - 행동의 로그 표준편차
            value: (batch_size, 1) - 상태 가치
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
    
    def get_action(
        self,
        state: torch.Tensor,
        deterministic: bool = False
    ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """
        행동 샘플링
        
        Args:
            state: (batch_size, input_dim)
            deterministic: True면 평균값 사용, False면 샘플링
        
        Returns:
            action: (batch_size, action_dim) - 선택된 행동
            log_prob: (batch_size,) - 로그 확률
            value: (batch_size, 1) - 상태 가치
        """
        from torch.distributions import Normal
        
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
    
    def evaluate_actions(
        self,
        state: torch.Tensor,
        action: torch.Tensor
    ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """
        행동 평가 (학습 시 사용)
        
        Args:
            state: (batch_size, input_dim)
            action: (batch_size, action_dim)
        
        Returns:
            log_prob: (batch_size,) - 로그 확률
            entropy: (batch_size,) - 엔트로피
            value: (batch_size, 1) - 상태 가치
        """
        from torch.distributions import Normal
        
        action_mean, action_log_std, value = self.forward(state)
        action_std = torch.exp(action_log_std)
        
        dist = Normal(action_mean, action_std)
        log_prob = dist.log_prob(action).sum(dim=-1)
        entropy = dist.entropy().sum(dim=-1)
        
        return log_prob, entropy, value


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
        num_layers: int = 2,
        dropout: float = 0.2
    ):
        """
        Args:
            input_dim: 입력 차원 (각 타임스텝의 feature 수)
            action_dim: 행동 차원
            hidden_dim: LSTM hidden 차원
            num_layers: LSTM 레이어 수
            dropout: Dropout 비율
        """
        super(RecurrentActorCritic, self).__init__()
        
        self.input_dim = input_dim
        self.action_dim = action_dim
        self.hidden_dim = hidden_dim
        self.num_layers = num_layers
        
        # LSTM for temporal features
        self.lstm = nn.LSTM(
            input_size=input_dim,
            hidden_size=hidden_dim,
            num_layers=num_layers,
            batch_first=True,
            dropout=dropout if num_layers > 1 else 0
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
        nn.init.constant_(self.actor_mean.bias, 0.0)
        nn.init.orthogonal_(self.actor_log_std.weight, gain=0.01)
        nn.init.constant_(self.actor_log_std.bias, 0.0)
        nn.init.orthogonal_(self.critic.weight, gain=1.0)
        nn.init.constant_(self.critic.bias, 0.0)
    
    def forward(
        self,
        state: torch.Tensor,
        hidden_state: Tuple[torch.Tensor, torch.Tensor] = None
    ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor, Tuple[torch.Tensor, torch.Tensor]]:
        """
        Forward pass with LSTM
        
        Args:
            state: (batch_size, seq_len, input_dim) or (batch_size, input_dim)
            hidden_state: (h_0, c_0) LSTM hidden state
                h_0: (num_layers, batch_size, hidden_dim)
                c_0: (num_layers, batch_size, hidden_dim)
        
        Returns:
            action_mean: (batch_size, action_dim)
            action_log_std: (batch_size, action_dim)
            value: (batch_size, 1)
            new_hidden_state: (h_n, c_n)
        """
        # state가 2D면 3D로 변환 (seq_len=1)
        if state.dim() == 2:
            state = state.unsqueeze(1)
        
        batch_size = state.size(0)
        
        # LSTM forward
        if hidden_state is None:
            # 초기 hidden state
            h_0 = torch.zeros(self.num_layers, batch_size, self.hidden_dim).to(state.device)
            c_0 = torch.zeros(self.num_layers, batch_size, self.hidden_dim).to(state.device)
            hidden_state = (h_0, c_0)
        
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
    
    def get_action(
        self,
        state: torch.Tensor,
        hidden_state: Tuple[torch.Tensor, torch.Tensor] = None,
        deterministic: bool = False
    ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor, Tuple[torch.Tensor, torch.Tensor]]:
        """
        행동 샘플링 (LSTM용)
        
        Args:
            state: (batch_size, seq_len, input_dim) or (batch_size, input_dim)
            hidden_state: LSTM hidden state
            deterministic: 결정적 행동 선택 여부
        
        Returns:
            action: (batch_size, action_dim)
            log_prob: (batch_size,)
            value: (batch_size, 1)
            new_hidden_state: (h_n, c_n)
        """
        from torch.distributions import Normal
        
        action_mean, action_log_std, value, new_hidden_state = self.forward(state, hidden_state)
        action_std = torch.exp(action_log_std)
        
        if deterministic:
            action = action_mean
            dist = Normal(action_mean, action_std)
            log_prob = dist.log_prob(action).sum(dim=-1)
        else:
            dist = Normal(action_mean, action_std)
            action = dist.sample()
            log_prob = dist.log_prob(action).sum(dim=-1)
        
        action = torch.clamp(action, -1.0, 1.0)
        
        return action, log_prob, value, new_hidden_state
    
    def evaluate_actions(
        self,
        state: torch.Tensor,
        action: torch.Tensor,
        hidden_state: Tuple[torch.Tensor, torch.Tensor] = None
    ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """
        행동 평가 (LSTM용)
        
        Args:
            state: (batch_size, seq_len, input_dim)
            action: (batch_size, action_dim)
            hidden_state: LSTM hidden state (optional)
        
        Returns:
            log_prob: (batch_size,)
            entropy: (batch_size,)
            value: (batch_size, 1)
        """
        from torch.distributions import Normal
        
        action_mean, action_log_std, value, _ = self.forward(state, hidden_state)
        action_std = torch.exp(action_log_std)
        
        dist = Normal(action_mean, action_std)
        log_prob = dist.log_prob(action).sum(dim=-1)
        entropy = dist.entropy().sum(dim=-1)
        
        return log_prob, entropy, value


class CNNActorCritic(nn.Module):
    """
    CNN 기반 Actor-Critic (이미지 입력용)
    주가 차트 이미지나 캔들스틱 차트를 입력으로 사용할 때 활용
    """
    
    def __init__(
        self,
        input_channels: int,
        action_dim: int,
        image_height: int = 84,
        image_width: int = 84
    ):
        """
        Args:
            input_channels: 입력 채널 수 (예: RGB=3)
            action_dim: 행동 차원
            image_height: 이미지 높이
            image_width: 이미지 너비
        """
        super(CNNActorCritic, self).__init__()
        
        # CNN feature extractor
        self.conv = nn.Sequential(
            nn.Conv2d(input_channels, 32, kernel_size=8, stride=4),
            nn.ReLU(),
            nn.Conv2d(32, 64, kernel_size=4, stride=2),
            nn.ReLU(),
            nn.Conv2d(64, 64, kernel_size=3, stride=1),
            nn.ReLU()
        )
        
        # CNN 출력 크기 계산
        def conv_output_size(size, kernel_size, stride):
            return (size - kernel_size) // stride + 1
        
        h = conv_output_size(conv_output_size(conv_output_size(image_height, 8, 4), 4, 2), 3, 1)
        w = conv_output_size(conv_output_size(conv_output_size(image_width, 8, 4), 4, 2), 3, 1)
        conv_output_dim = 64 * h * w
        
        # Fully connected layers
        self.fc = nn.Sequential(
            nn.Linear(conv_output_dim, 512),
            nn.ReLU()
        )
        
        # Actor head
        self.actor_mean = nn.Linear(512, action_dim)
        self.actor_log_std = nn.Linear(512, action_dim)
        
        # Critic head
        self.critic = nn.Linear(512, 1)
        
        self._initialize_weights()
    
    def _initialize_weights(self):
        """가중치 초기화"""
        for m in self.modules():
            if isinstance(m, (nn.Conv2d, nn.Linear)):
                nn.init.orthogonal_(m.weight, gain=np.sqrt(2))
                nn.init.constant_(m.bias, 0.0)
        
        nn.init.orthogonal_(self.actor_mean.weight, gain=0.01)
        nn.init.constant_(self.actor_mean.bias, 0.0)
    
    def forward(self, state: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """
        Forward pass
        
        Args:
            state: (batch_size, channels, height, width)
        
        Returns:
            action_mean, action_log_std, value
        """
        # CNN features
        conv_features = self.conv(state)
        conv_features = conv_features.view(conv_features.size(0), -1)
        
        # FC features
        features = self.fc(conv_features)
        
        # Actor
        action_mean = torch.tanh(self.actor_mean(features))
        action_log_std = self.actor_log_std(features)
        action_log_std = torch.clamp(action_log_std, -20, 2)
        
        # Critic
        value = self.critic(features)
        
        return action_mean, action_log_std, value