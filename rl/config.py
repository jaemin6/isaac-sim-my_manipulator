# rl/config.py (V3 - 보수적 설정)
"""
강화학습 설정 V3 - 좋은 거래 학습
"""

from dataclasses import dataclass
from typing import List, Optional


@dataclass
class EnvironmentConfig:
    """트레이딩 환경 설정"""
    
    window_size: int = 60
    initial_balance: float = 10000.0
    transaction_fee: float = 0.001
    max_position: float = 1.0
    
    # 🆕 V3: 거래 제어
    min_hold_steps: int = 5  # 최소 홀딩 시간
    position_change_threshold: float = 0.3  # 포지션 변경 임계값
    
    data_dir: str = "data/processed"
    train_ratio: float = 0.7
    val_ratio: float = 0.15
    test_ratio: float = 0.15


@dataclass
class NetworkConfig:
    """신경망 구조 설정"""
    
    hidden_dims: List[int] = None
    activation: str = 'relu'
    use_recurrent: bool = False
    lstm_hidden_dim: int = 128
    lstm_num_layers: int = 2
    dropout: float = 0.2
    
    def __post_init__(self):
        if self.hidden_dims is None:
            self.hidden_dims = [256, 128, 64]


@dataclass
class PPOConfig:
    """PPO 알고리즘 하이퍼파라미터"""
    
    learning_rate: float = 3e-4
    lr_schedule: str = 'constant'
    
    gamma: float = 0.99
    gae_lambda: float = 0.95
    clip_epsilon: float = 0.2
    
    # 🔥 V3: 엔트로피 대폭 감소
    value_loss_coef: float = 0.5
    entropy_coef: float = 0.02  # 🔥 0.1 → 0.02 (5배 감소!)
    
    max_grad_norm: float = 0.5
    
    n_epochs: int = 10
    batch_size: int = 64
    n_steps: int = 2048


@dataclass
class TrainingConfig:
    """학습 설정"""
    
    max_episodes: int = 1000
    max_steps_per_episode: int = 2000
    
    log_interval: int = 10
    save_interval: int = 50
    eval_interval: int = 20
    
    run_diagnostic: bool = True
    check_actions_every: int = 100
    
    early_stopping: bool = True
    patience: int = 300  # 🔥 200 → 300 (더 오래 기다림)
    min_improvement: float = 0.01
    
    checkpoint_dir: str = "checkpoints/rl"
    best_model_path: str = "checkpoints/rl/best_model.pt"
    
    device: str = 'cuda'
    seed: Optional[int] = 42


@dataclass
class EvaluationConfig:
    """평가 설정"""
    
    n_eval_episodes: int = 10
    deterministic: bool = True
    
    save_trades: bool = True
    plot_results: bool = True
    
    calculate_sharpe: bool = True
    calculate_sortino: bool = True
    calculate_max_drawdown: bool = True


@dataclass
class RLConfig:
    """전체 강화학습 설정"""
    
    env: EnvironmentConfig = None
    network: NetworkConfig = None
    ppo: PPOConfig = None
    training: TrainingConfig = None
    evaluation: EvaluationConfig = None
    
    def __post_init__(self):
        if self.env is None:
            self.env = EnvironmentConfig()
        if self.network is None:
            self.network = NetworkConfig()
        if self.ppo is None:
            self.ppo = PPOConfig()
        if self.training is None:
            self.training = TrainingConfig()
        if self.evaluation is None:
            self.evaluation = EvaluationConfig()
    
    @classmethod
    def default(cls):
        """V3 기본 설정 (보수적)"""
        return cls()
    
    @classmethod
    def conservative(cls):
        """매우 보수적 설정"""
        config = cls()
        config.ppo.learning_rate = 1e-4
        config.ppo.clip_epsilon = 0.1
        config.ppo.entropy_coef = 0.01
        config.env.max_position = 0.5
        config.env.min_hold_steps = 10  # 더 긴 홀딩
        return config
    
    @classmethod
    def aggressive(cls):
        """공격적 설정"""
        config = cls()
        config.ppo.learning_rate = 5e-4
        config.ppo.clip_epsilon = 0.3
        config.ppo.entropy_coef = 0.05
        config.env.max_position = 1.5
        config.env.min_hold_steps = 3  # 더 짧은 홀딩
        return config
    
    @classmethod
    def balanced(cls):
        """균형잡힌 설정"""
        config = cls()
        config.ppo.entropy_coef = 0.03
        config.env.min_hold_steps = 7
        return config
    
    def to_dict(self):
        return {
            'environment': self.env.__dict__,
            'network': self.network.__dict__,
            'ppo': self.ppo.__dict__,
            'training': self.training.__dict__,
            'evaluation': self.evaluation.__dict__
        }
    
    def print_config(self):
        print("=" * 60)
        print("강화학습 설정 V3 (좋은 거래 학습)")
        print("=" * 60)
        
        print("\n[환경 설정]")
        for key, value in self.env.__dict__.items():
            print(f"  {key}: {value}")
        
        print("\n[네트워크 설정]")
        for key, value in self.network.__dict__.items():
            print(f"  {key}: {value}")
        
        print("\n[PPO 설정]")
        for key, value in self.ppo.__dict__.items():
            print(f"  {key}: {value}")
        
        print("\n[학습 설정]")
        for key, value in self.training.__dict__.items():
            print(f"  {key}: {value}")
        
        print("\n[평가 설정]")
        for key, value in self.evaluation.__dict__.items():
            print(f"  {key}: {value}")
        
        print("=" * 60)


# 사전 정의된 설정들
DEFAULT_CONFIG = RLConfig.default()
CONSERVATIVE_CONFIG = RLConfig.conservative()
AGGRESSIVE_CONFIG = RLConfig.aggressive()
BALANCED_CONFIG = RLConfig.balanced()

# 하위 호환성
CURRICULUM_CONFIG = BALANCED_CONFIG
RECURRENT_CONFIG = BALANCED_CONFIG


def get_config(config_name: str = 'default') -> RLConfig:
    """설정 로드"""
    configs = {
        'default': DEFAULT_CONFIG,
        'conservative': CONSERVATIVE_CONFIG,
        'aggressive': AGGRESSIVE_CONFIG,
        'balanced': BALANCED_CONFIG,
        'curriculum': BALANCED_CONFIG,
        'recurrent': BALANCED_CONFIG,
    }
    
    if config_name not in configs:
        print(f"⚠️  Unknown config: {config_name}, using default")
        return DEFAULT_CONFIG
    
    return configs[config_name]


if __name__ == '__main__':
    print("\nV3 기본 설정:")
    DEFAULT_CONFIG.print_config()