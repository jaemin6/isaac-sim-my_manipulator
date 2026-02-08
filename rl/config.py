# rl/config.py
"""
강화학습 설정 파일

모든 하이퍼파라미터와 환경 설정을 관리합니다.
"""

from dataclasses import dataclass
from typing import List, Optional


@dataclass
class EnvironmentConfig:
    """트레이딩 환경 설정"""
    
    # 데이터 설정
    window_size: int = 60  # 관찰 윈도우 크기 (60분)
    
    # 계정 설정
    initial_balance: float = 10000.0  # 초기 자본금 ($10,000)
    transaction_fee: float = 0.001  # 거래 수수료 (0.1%)
    max_position: float = 1.0  # 최대 포지션 크기 (자본금 대비)
    
    # 데이터 경로
    data_dir: str = "data/processed"
    
    # 학습/검증/테스트 분할 비율
    train_ratio: float = 0.7
    val_ratio: float = 0.15
    test_ratio: float = 0.15


@dataclass
class NetworkConfig:
    """신경망 구조 설정"""
    
    # 네트워크 아키텍처
    hidden_dims: List[int] = None  # [256, 128, 64]
    activation: str = 'relu'  # 'relu', 'tanh', 'elu'
    
    # Recurrent 설정
    use_recurrent: bool = False  # LSTM 사용 여부
    lstm_hidden_dim: int = 128
    lstm_num_layers: int = 2
    
    # Dropout
    dropout: float = 0.2
    
    def __post_init__(self):
        if self.hidden_dims is None:
            self.hidden_dims = [256, 128, 64]


@dataclass
class PPOConfig:
    """PPO 알고리즘 하이퍼파라미터"""
    
    # 학습률
    learning_rate: float = 3e-4
    lr_schedule: str = 'constant'  # 'constant', 'linear', 'exponential'
    
    # PPO 파라미터
    gamma: float = 0.99  # 할인율
    gae_lambda: float = 0.95  # GAE lambda
    clip_epsilon: float = 0.2  # PPO clipping parameter
    
    # 손실 함수 계수
    value_loss_coef: float = 0.5  # 가치 손실 계수
    entropy_coef: float = 0.01  # 엔트로피 보너스 계수
    
    # 그래디언트
    max_grad_norm: float = 0.5  # 그래디언트 클리핑
    
    # 학습 설정
    n_epochs: int = 10  # PPO 업데이트 에폭 수
    batch_size: int = 64  # 미니배치 크기
    n_steps: int = 2048  # 업데이트당 스텝 수


@dataclass
class TrainingConfig:
    """학습 설정"""
    
    # 에피소드 설정
    max_episodes: int = 1000  # 최대 에피소드 수
    max_steps_per_episode: int = 10000  # 에피소드당 최대 스텝
    
    # 로깅
    log_interval: int = 10  # 로그 출력 간격 (에피소드)
    save_interval: int = 50  # 모델 저장 간격 (에피소드)
    eval_interval: int = 20  # 평가 간격 (에피소드)
    
    # 조기 종료
    early_stopping: bool = True
    patience: int = 300  # 성능 개선이 없을 때 기다리는 에피소드 수
    min_improvement: float = 0.01  # 최소 개선폭
    
    # 체크포인트
    checkpoint_dir: str = "checkpoints/rl"
    best_model_path: str = "checkpoints/rl/best_model.pt"
    
    # 디바이스
    device: str = 'cuda'  # 'cuda' or 'cpu'
    
    # 시드
    seed: Optional[int] = 42


@dataclass
class EvaluationConfig:
    """평가 설정"""
    
    n_eval_episodes: int = 10  # 평가 에피소드 수
    deterministic: bool = True  # 결정적 행동 선택
    
    # 백테스팅
    save_trades: bool = True  # 거래 내역 저장
    plot_results: bool = True  # 결과 시각화
    
    # 성능 지표
    calculate_sharpe: bool = True
    calculate_sortino: bool = True
    calculate_max_drawdown: bool = True


@dataclass
class RLConfig:
    """전체 강화학습 설정 (모든 설정 통합)"""
    
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
        """기본 설정 반환"""
        return cls()
    
    @classmethod
    def conservative(cls):
        """보수적인 설정 (안정적인 학습)"""
        config = cls()
        config.ppo.learning_rate = 3e-4
        config.ppo.clip_epsilon = 0.2
        config.ppo.entropy_coef = 0.01
        config.env.max_position = 0.5
        return config
    
    @classmethod
    def aggressive(cls):
        """공격적인 설정 (빠른 학습, 높은 리스크)"""
        config = cls()
        config.ppo.learning_rate = 1e-3
        config.ppo.clip_epsilon = 0.3
        config.ppo.entropy_coef = 0.05
        config.env.max_position = 2.0
        return config
    
    @classmethod
    def recurrent(cls):
        """LSTM 기반 설정"""
        config = cls()
        config.network.use_recurrent = True
        config.network.lstm_hidden_dim = 256
        config.network.lstm_num_layers = 3
        return config
    
    def to_dict(self):
        """딕셔너리로 변환"""
        return {
            'environment': self.env.__dict__,
            'network': self.network.__dict__,
            'ppo': self.ppo.__dict__,
            'training': self.training.__dict__,
            'evaluation': self.evaluation.__dict__
        }
    
    def print_config(self):
        """설정 출력"""
        print("=" * 60)
        print("강화학습 설정")
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
RECURRENT_CONFIG = RLConfig.recurrent()


# 설정 로드 함수
def get_config(config_name: str = 'default') -> RLConfig:
    """
    설정 이름으로 설정 로드
    
    Args:
        config_name: 'default', 'conservative', 'aggressive', 'recurrent'
    
    Returns:
        RLConfig 객체
    """
    configs = {
        'default': DEFAULT_CONFIG,
        'conservative': CONSERVATIVE_CONFIG,
        'aggressive': AGGRESSIVE_CONFIG,
        'recurrent': RECURRENT_CONFIG
    }
    
    if config_name not in configs:
        print(f"⚠️  Unknown config: {config_name}, using default")
        return DEFAULT_CONFIG
    
    return configs[config_name]


if __name__ == '__main__':
    # 설정 테스트
    print("\n기본 설정:")
    DEFAULT_CONFIG.print_config()
    
    print("\n\n보수적 설정:")
    CONSERVATIVE_CONFIG.print_config()