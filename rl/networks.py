import torch
import torch.nn as nn
from typing import List, Tuple

# 1. MLP 기반 Actor-Critic (표준 데이터용)
class ActorCritic(nn.Module):
    def __init__(self, input_dim: int, action_dim: int, hidden_dims: List[int] = [256, 128, 64], activation: str = 'relu'):
        super(ActorCritic, self).__init__()
        # 1. Feature Extractor (Shared)
        # 2. Actor Backbone & Heads (Mean, Log_std)
        # 3. Critic Backbone & Head (Value)
        pass

    def _initialize_weights(self):
        """Orthogonal 초기화 로직"""
        pass

    def forward(self, state: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """Mean, Log_std, Value 반환"""
        pass

    def get_action(self, state: torch.Tensor, deterministic: bool = False) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """행동 샘플링 및 Log_prob 계산"""
        pass

    def evaluate_actions(self, state: torch.Tensor, action: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """학습을 위한 log_prob, entropy, value 평가"""
        pass


# 2. LSTM 기반 Recurrent Actor-Critic (시계열 데이터용)
class RecurrentActorCritic(nn.Module):
    def __init__(self, input_dim: int, action_dim: int, hidden_dim: int = 128, num_layers: int = 2, dropout: float = 0.2):
        super(RecurrentActorCritic, self).__init__()
        # 1. LSTM Layer
        # 2. Actor Heads
        # 3. Critic Head
        pass

    def _initialize_weights(self):
        pass

    def forward(self, state: torch.Tensor, hidden_state: Tuple[torch.Tensor, torch.Tensor] = None):
        """Hidden state를 포함한 시퀀스 처리"""
        pass

    def get_action(self, state: torch.Tensor, hidden_state: Tuple[torch.Tensor, torch.Tensor] = None, deterministic: bool = False):
        pass

    def evaluate_actions(self, state: torch.Tensor, action: torch.Tensor, hidden_state: Tuple[torch.Tensor, torch.Tensor] = None):
        pass


# 3. CNN 기반 Actor-Critic (이미지 데이터용)
class CNNActorCritic(nn.Module):
    def __init__(self, input_channels: int, action_dim: int, image_height: int = 84, image_width: int = 84):
        super(CNNActorCritic, self).__init__()
        # 1. Conv2d Layers
        # 2. Fully Connected Layers
        # 3. Actor/Critic Heads
        pass

    def _initialize_weights(self):
        pass

    def forward(self, state: torch.Tensor):
        pass