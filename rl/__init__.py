# rl/__init__.py (V2 - 수정)
"""
강화학습 모듈 초기화
"""

from .environment import TradingEnvironment, CurriculumTradingEnv, MultiAssetTradingEnvironment
from .ppo_agent import PPOAgent
from .config import (
    RLConfig,
    EnvironmentConfig,
    NetworkConfig,
    PPOConfig,
    TrainingConfig,
    EvaluationConfig,
    get_config,
    DEFAULT_CONFIG,
    CONSERVATIVE_CONFIG,
    AGGRESSIVE_CONFIG,
    CURRICULUM_CONFIG  # 🔥 RECURRENT_CONFIG → CURRICULUM_CONFIG
)

__all__ = [
    # Environment
    'TradingEnvironment',
    'CurriculumTradingEnv',
    'MultiAssetTradingEnvironment',
    
    # Agent
    'PPOAgent',
    
    # Config
    'RLConfig',
    'EnvironmentConfig',
    'NetworkConfig',
    'PPOConfig',
    'TrainingConfig',
    'EvaluationConfig',
    'get_config',
    
    # Presets
    'DEFAULT_CONFIG',
    'CONSERVATIVE_CONFIG',
    'AGGRESSIVE_CONFIG',
    'CURRICULUM_CONFIG',  # 🔥 RECURRENT_CONFIG → CURRICULUM_CONFIG
]