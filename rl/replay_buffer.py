# rl/replay_buffer.py
import numpy as np
from typing import List, Dict, Tuple


class RolloutBuffer:
    """
    PPO를 위한 Rollout Buffer
    
    On-policy 알고리즘인 PPO는 현재 정책으로 수집한 데이터만 사용합니다.
    따라서 경험을 장기간 저장하지 않고, 한 번의 업데이트 후 버퍼를 비웁니다.
    """
    
    def __init__(self, buffer_size: int = 2048):
        """
        Args:
            buffer_size: 버퍼 크기 (스텝 수)
        """
        self.buffer_size = buffer_size
        self.reset()
    
    def reset(self):
        """버퍼 초기화"""
        self.states = []
        self.actions = []
        self.rewards = []
        self.values = []
        self.log_probs = []
        self.dones = []
        self.ptr = 0
        self.trajectory_start_idx = 0
    
    def add(
        self,
        state: np.ndarray,
        action: np.ndarray,
        reward: float,
        value: float,
        log_prob: float,
        done: bool
    ):
        """
        경험 추가
        
        Args:
            state: 상태
            action: 행동
            reward: 보상
            value: 상태 가치
            log_prob: 로그 확률
            done: 에피소드 종료 여부
        """
        self.states.append(state)
        self.actions.append(action)
        self.rewards.append(reward)
        self.values.append(value)
        self.log_probs.append(log_prob)
        self.dones.append(done)
        self.ptr += 1
    
    def get(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        버퍼의 모든 데이터 반환
        
        Returns:
            states: (N, state_dim)
            actions: (N, action_dim)
            rewards: (N,)
            values: (N,)
            log_probs: (N,)
            dones: (N,)
        """
        return (
            np.array(self.states, dtype=np.float32),
            np.array(self.actions, dtype=np.float32),
            np.array(self.rewards, dtype=np.float32),
            np.array(self.values, dtype=np.float32),
            np.array(self.log_probs, dtype=np.float32),
            np.array(self.dones, dtype=np.float32)
        )
    
    def is_full(self) -> bool:
        """버퍼가 가득 찼는지 확인"""
        return self.ptr >= self.buffer_size
    
    def size(self) -> int:
        """현재 버퍼에 저장된 경험 수"""
        return self.ptr
    
    def get_last_values(self, n: int = 10) -> List[float]:
        """마지막 n개의 가치 반환"""
        return self.values[-n:]
    
    def get_last_rewards(self, n: int = 10) -> List[float]:
        """마지막 n개의 보상 반환"""
        return self.rewards[-n:]


class EpisodeBuffer:
    """
    에피소드 단위로 경험을 저장하는 버퍼
    """
    
    def __init__(self):
        """초기화"""
        self.episodes = []
        self.current_episode = {
            'states': [],
            'actions': [],
            'rewards': [],
            'values': [],
            'log_probs': [],
            'dones': []
        }
    
    def add(
        self,
        state: np.ndarray,
        action: np.ndarray,
        reward: float,
        value: float,
        log_prob: float,
        done: bool
    ):
        """현재 에피소드에 경험 추가"""
        self.current_episode['states'].append(state)
        self.current_episode['actions'].append(action)
        self.current_episode['rewards'].append(reward)
        self.current_episode['values'].append(value)
        self.current_episode['log_probs'].append(log_prob)
        self.current_episode['dones'].append(done)
        
        # 에피소드가 끝나면 저장
        if done:
            self.episodes.append(self.current_episode)
            self.current_episode = {
                'states': [],
                'actions': [],
                'rewards': [],
                'values': [],
                'log_probs': [],
                'dones': []
            }
    
    def get_all_episodes(self) -> List[Dict]:
        """모든 에피소드 반환"""
        return self.episodes
    
    def get_last_episode(self) -> Dict:
        """마지막 에피소드 반환"""
        if len(self.episodes) > 0:
            return self.episodes[-1]
        return None
    
    def get_episode_returns(self) -> List[float]:
        """각 에피소드의 총 보상 반환"""
        returns = []
        for episode in self.episodes:
            total_reward = sum(episode['rewards'])
            returns.append(total_reward)
        return returns
    
    def get_episode_lengths(self) -> List[int]:
        """각 에피소드의 길이 반환"""
        lengths = []
        for episode in self.episodes:
            lengths.append(len(episode['rewards']))
        return lengths
    
    def flatten_episodes(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        모든 에피소드를 flatten하여 반환
        
        Returns:
            states, actions, rewards, values, log_probs, dones
        """
        all_states = []
        all_actions = []
        all_rewards = []
        all_values = []
        all_log_probs = []
        all_dones = []
        
        for episode in self.episodes:
            all_states.extend(episode['states'])
            all_actions.extend(episode['actions'])
            all_rewards.extend(episode['rewards'])
            all_values.extend(episode['values'])
            all_log_probs.extend(episode['log_probs'])
            all_dones.extend(episode['dones'])
        
        return (
            np.array(all_states, dtype=np.float32),
            np.array(all_actions, dtype=np.float32),
            np.array(all_rewards, dtype=np.float32),
            np.array(all_values, dtype=np.float32),
            np.array(all_log_probs, dtype=np.float32),
            np.array(all_dones, dtype=np.float32)
        )
    
    def reset(self):
        """버퍼 초기화"""
        self.episodes = []
        self.current_episode = {
            'states': [],
            'actions': [],
            'rewards': [],
            'values': [],
            'log_probs': [],
            'dones': []
        }
    
    def num_episodes(self) -> int:
        """저장된 에피소드 수"""
        return len(self.episodes)


class PrioritizedReplayBuffer:
    """
    우선순위 경험 재생 버퍼 (Off-policy 알고리즘용)
    
    참고: PPO는 on-policy이므로 일반적으로 사용하지 않지만,
    실험적으로 중요한 경험을 재사용하고 싶을 때 활용 가능
    """
    
    def __init__(self, capacity: int = 10000, alpha: float = 0.6, beta: float = 0.4):
        """
        Args:
            capacity: 버퍼 용량
            alpha: 우선순위 지수 (0 = uniform, 1 = full prioritization)
            beta: Importance sampling 지수
        """
        self.capacity = capacity
        self.alpha = alpha
        self.beta = beta
        self.buffer = []
        self.priorities = np.zeros(capacity, dtype=np.float32)
        self.ptr = 0
        self.size = 0
    
    def add(
        self,
        state: np.ndarray,
        action: np.ndarray,
        reward: float,
        next_state: np.ndarray,
        done: bool,
        priority: float = None
    ):
        """경험 추가"""
        experience = (state, action, reward, next_state, done)
        
        # 우선순위 설정 (제공되지 않으면 최대 우선순위 사용)
        if priority is None:
            priority = self.priorities[:self.size].max() if self.size > 0 else 1.0
        
        if len(self.buffer) < self.capacity:
            self.buffer.append(experience)
        else:
            self.buffer[self.ptr] = experience
        
        self.priorities[self.ptr] = priority
        self.ptr = (self.ptr + 1) % self.capacity
        self.size = min(self.size + 1, self.capacity)
    
    def sample(self, batch_size: int) -> Tuple:
        """우선순위 기반 샘플링"""
        if self.size == 0:
            return None
        
        # 우선순위 기반 확률 계산
        priorities = self.priorities[:self.size] ** self.alpha
        probs = priorities / priorities.sum()
        
        # 샘플링
        indices = np.random.choice(self.size, batch_size, p=probs, replace=False)
        
        # Importance sampling weights
        weights = (self.size * probs[indices]) ** (-self.beta)
        weights /= weights.max()  # Normalize
        
        # 배치 생성
        batch = [self.buffer[idx] for idx in indices]
        states, actions, rewards, next_states, dones = zip(*batch)
        
        return (
            np.array(states, dtype=np.float32),
            np.array(actions, dtype=np.float32),
            np.array(rewards, dtype=np.float32),
            np.array(next_states, dtype=np.float32),
            np.array(dones, dtype=np.float32),
            indices,
            weights
        )
    
    def update_priorities(self, indices: np.ndarray, priorities: np.ndarray):
        """우선순위 업데이트"""
        for idx, priority in zip(indices, priorities):
            self.priorities[idx] = priority
    
    def __len__(self):
        return self.size