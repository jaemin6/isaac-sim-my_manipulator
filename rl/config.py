from dataclasses import dataclass
from typing import List, Optional

@dataclass
class EnvironmentConfig:
    """데이터, 계정, 데이터 분할 관련 설정"""
    window_size: int = 60
    initial_balance: float = 10000.0
    # ... (기타 환경 변수)

@dataclass
class NetworkConfig:
    """신경망 아키텍처(MLP, LSTM) 및 드롭아웃 설정"""
    hidden_dims: List[int] = None
    use_recurrent: bool = False
    # ... (네트워크 레이어 정보)

@dataclass
class PPOConfig:
    """PPO 알고리즘 전용 하이퍼파라미터 (gamma, lambda, clip 등)"""
    learning_rate: float = 3e-4
    gamma: float = 0.99
    # ... (PPO 핵심 파라미터)

@dataclass
class TrainingConfig:
    """학습 프로세스(에피소드, 로깅, 체크포인트) 설정"""
    max_episodes: int = 1000
    device: str = 'cuda'
    # ... (저장 및 조기종료 설정)

@dataclass
class RLConfig:
    """위의 모든 설정을 통합하고 전략별(보수/공격 등) 팩토리 메서드 제공"""
    env: EnvironmentConfig = None
    network: NetworkConfig = None
    ppo: PPOConfig = None
    # ... 

    @classmethod
    def default(cls): ...      # 기본값 생성
    @classmethod
    def conservative(cls): ... # 안정 지향 설정
    @classmethod
    def aggressive(cls): ...   # 수익 극대화 설정

def get_config(config_name: str = 'default') -> RLConfig:
    """이름에 따른 설정 객체 반환 함수"""
    ...