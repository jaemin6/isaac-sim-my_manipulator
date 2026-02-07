import numpy as np
from typing import List, Dict, Tuple

class RolloutBuffer:
    """PPO 등 On-policy 알고리즘용 (고정된 스텝 수 수집 후 초기화)"""
    def __init__(self, buffer_size: int = 2048): ...
    def reset(self): ... # 한 번의 업데이트 후 버퍼 비우기
    def add(self, state, action, reward, value, log_prob, done): ... # 스텝 데이터 저장
    def get(self) -> Tuple: ... # 저장된 모든 데이터를 넘파이 배열로 반환
    def is_full(self) -> bool: ...
    def size(self) -> int: ...

class EpisodeBuffer:
    """에피소드 단위(시작~종료) 통계 및 저장용"""
    def __init__(self): ...
    def add(self, state, action, reward, value, log_prob, done): ... # 에피소드 종료 시 자동 리스트 저장
    def get_all_episodes(self) -> List[Dict]: ...
    def get_episode_returns(self) -> List[float]: ... # 에피소드별 총 보상 합계
    def flatten_episodes(self) -> Tuple: ... # 여러 에피소드를 하나로 합쳐서 반환
    def reset(self): ...

class PrioritizedReplayBuffer:
    """DQN, SAC 등 Off-policy 알고리즘용 (우선순위 기반 샘플링)"""
    def __init__(self, capacity: int = 10000, alpha: float = 0.6, beta: float = 0.4): ...
    def add(self, state, action, reward, next_state, done, priority=None): ...
    def sample(self, batch_size: int) -> Tuple: ... # 중요도(TD-error)가 높은 데이터 우선 추출
    def update_priorities(self, indices, priorities): ... # 학습 후 새로운 우선순위 반영