# rl/environment.py
import numpy as np
import torch
from typing import Dict, Tuple, Optional
import gym
from gym import spaces


class TradingEnvironment(gym.Env):
    """
    암호화폐 트레이딩을 위한 강화학습 환경
    OpenAI Gym 인터페이스 구현
    """
    
    def __init__(
        self,
        data: np.ndarray,
        initial_balance: float = 10000.0,
        transaction_fee: float = 0.001,
        window_size: int = 60,
        max_position: float = 1.0
    ):
        """
        Args:
            data: 시계열 데이터 (n_samples, n_features)
            initial_balance: 초기 자본금
            transaction_fee: 거래 수수료 비율
            window_size: 관찰 윈도우 크기
            max_position: 최대 포지션 크기 (자본금 대비 비율)
        """
        super(TradingEnvironment, self).__init__()
        
        self.data = data
        self.initial_balance = initial_balance
        self.transaction_fee = transaction_fee
        self.window_size = window_size
        self.max_position = max_position
        
        # 환경 상태
        self.current_step = 0
        self.balance = initial_balance
        self.position = 0.0  # 현재 보유 포지션 (-1 ~ 1)
        self.entry_price = 0.0
        self.total_profit = 0.0
        self.trade_history = []
        
        # Action space: [-1, 1] 연속 행동 (매도 ~ 매수)
        self.action_space = spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(1,),
            dtype=np.float32
        )
        
        # Observation space: 시장 데이터 + 계정 정보
        n_features = data.shape[1]
        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(window_size, n_features + 3),  # +3 for balance, position, profit
            dtype=np.float32
        )
        
    def reset(self) -> np.ndarray:
        """환경 리셋"""
        self.current_step = self.window_size
        self.balance = self.initial_balance
        self.position = 0.0
        self.entry_price = 0.0
        self.total_profit = 0.0
        self.trade_history = []
        
        return self._get_observation()
    
    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, Dict]:
        """
        환경 스텝 실행
        
        Args:
            action: 행동 값 [-1, 1]
        
        Returns:
            observation: 다음 상태
            reward: 보상
            done: 에피소드 종료 여부
            info: 추가 정보
        """
        action = np.clip(action[0], -1.0, 1.0)
        
        # 현재 가격
        current_price = self._get_current_price()
        
        # 행동 실행 (포지션 변경)
        reward = self._execute_action(action, current_price)
        
        # 다음 스텝으로 이동
        self.current_step += 1
        
        # 종료 조건 확인
        done = self.current_step >= len(self.data) - 1
        if self.balance <= 0:
            done = True
            reward -= 100  # 파산 페널티
        
        # 다음 관찰
        observation = self._get_observation()
        
        # 추가 정보
        info = {
            'balance': self.balance,
            'position': self.position,
            'total_profit': self.total_profit,
            'current_price': current_price,
            'num_trades': len(self.trade_history)
        }
        
        return observation, reward, done, info
    
    def _execute_action(self, action: float, current_price: float) -> float:
        """
        행동 실행 및 보상 계산 (재설계된 보상 함수)
        
        Args:
            action: 행동 값 [-1, 1]
            current_price: 현재 가격
        
        Returns:
            reward: 보상
        """
        # 목표 포지션 계산
        target_position = action * self.max_position
        position_change = target_position - self.position
        
        reward = 0.0
        prev_position = self.position
        
        # 포지션 변경이 있는 경우
        if abs(position_change) > 0.01:
            # 거래 실행
            trade_amount = abs(position_change) * self.balance
            fee = trade_amount * self.transaction_fee
            
            # 거래 자체에 작은 긍정적 보상 (탐험 장려)
            reward += 1.0  # 0.5 → 1.0
            
            # 포지션 종료 시 손익 계산
            if self.position != 0:
                if self.position > 0:  # 롱 포지션 종료
                    profit = (current_price - self.entry_price) * abs(self.position) * self.balance
                else:  # 숏 포지션 종료
                    profit = (self.entry_price - current_price) * abs(self.position) * self.balance
                
                profit -= fee
                self.balance += profit
                self.total_profit += profit
                
                # 수익률을 보상으로 (스케일 대폭 증가: 1000 → 5000)
                profit_ratio = profit / self.initial_balance
                reward += profit_ratio * 5000  # 5배 증가!
                
                # 거래 기록
                self.trade_history.append({
                    'step': self.current_step,
                    'type': 'close',
                    'position': self.position,
                    'entry_price': self.entry_price,
                    'exit_price': current_price,
                    'profit': profit
                })
            
            # 새로운 포지션 진입
            self.position = target_position
            self.entry_price = current_price
            self.balance -= fee
            
            # 거래 기록
            if abs(self.position) > 0.01:
                self.trade_history.append({
                    'step': self.current_step,
                    'type': 'open',
                    'position': self.position,
                    'entry_price': self.entry_price
                })
        
        # 보유 중인 포지션의 미실현 손익 (보상 스케일 증가: 100 → 500)
        if self.position != 0:
            if self.position > 0:
                price_change = (current_price - self.entry_price) / self.entry_price
                unrealized_pnl = price_change * abs(self.position) * self.initial_balance
            else:
                price_change = (self.entry_price - current_price) / self.entry_price
                unrealized_pnl = price_change * abs(self.position) * self.initial_balance
            
            # 미실현 손익을 큰 보상으로 (5배 증가)
            reward += unrealized_pnl / self.initial_balance * 500
            
            # 포지션 방향이 맞으면 추가 보상
            if unrealized_pnl > 0:
                reward += 2.0  # 1.0 → 2.0 올바른 방향 보너스
        else:
            # Hold 패널티 강화 (아무것도 안 하는 걸 막기)
            reward -= 1.0  # 0.5 → 1.0 (2배 증가)
        
        # 잔액이 크게 줄어들면 페널티
        balance_ratio = self.balance / self.initial_balance
        if balance_ratio < 0.5:  # 50% 이상 손실
            reward -= 20.0  # 10.0 → 20.0
        elif balance_ratio < 0.8:  # 20% 이상 손실
            reward -= 5.0   # 2.0 → 5.0
        
        # 총 수익이 증가하면 보상
        if self.total_profit > 0:
            reward += self.total_profit / self.initial_balance * 50  # 10 → 50
        
        return reward
    
    def _get_observation(self) -> np.ndarray:
        """
        현재 관찰 상태 반환
        
        Returns:
            observation: (window_size, n_features + 3)
        """
        # 시장 데이터 윈도우
        start_idx = self.current_step - self.window_size
        end_idx = self.current_step
        market_data = self.data[start_idx:end_idx]
        
        # 계정 정보 추가
        balance_normalized = self.balance / self.initial_balance
        position_info = self.position
        profit_normalized = self.total_profit / self.initial_balance
        
        account_info = np.array([
            balance_normalized,
            position_info,
            profit_normalized
        ])
        
        # 계정 정보를 각 타임스텝에 추가
        account_info_expanded = np.tile(account_info, (self.window_size, 1))
        
        # 결합
        observation = np.concatenate([market_data, account_info_expanded], axis=1)
        
        return observation.astype(np.float32)
    
    def _get_current_price(self) -> float:
        """
        현재 종가 반환 (첫 번째 feature가 close price라고 가정)
        
        Returns:
            current_price: 현재 가격
        """
        return self.data[self.current_step, 0]
    
    def render(self, mode='human'):
        """환경 시각화"""
        if mode == 'human':
            profit_pct = (self.total_profit / self.initial_balance) * 100
            print(f"Step: {self.current_step}")
            print(f"Balance: ${self.balance:.2f}")
            print(f"Position: {self.position:.2f}")
            print(f"Total Profit: ${self.total_profit:.2f} ({profit_pct:.2f}%)")
            print(f"Trades: {len(self.trade_history)}")
            print("-" * 50)
    
    def get_portfolio_value(self) -> float:
        """현재 포트폴리오 가치 계산"""
        current_price = self._get_current_price()
        
        # 현금 + 포지션 가치
        if self.position != 0:
            if self.position > 0:
                position_value = (current_price - self.entry_price) * abs(self.position) * self.balance
            else:
                position_value = (self.entry_price - current_price) * abs(self.position) * self.balance
        else:
            position_value = 0
        
        return self.balance + position_value


class MultiAssetTradingEnvironment(TradingEnvironment):
    """
    다중 자산 트레이딩 환경
    여러 암호화폐를 동시에 거래할 수 있는 환경
    """
    
    def __init__(
        self,
        data_dict: Dict[str, np.ndarray],
        initial_balance: float = 10000.0,
        transaction_fee: float = 0.001,
        window_size: int = 60,
        max_position_per_asset: float = 0.5
    ):
        """
        Args:
            data_dict: 자산별 시계열 데이터 딕셔너리
            initial_balance: 초기 자본금
            transaction_fee: 거래 수수료 비율
            window_size: 관찰 윈도우 크기
            max_position_per_asset: 자산당 최대 포지션 크기
        """
        self.data_dict = data_dict
        self.assets = list(data_dict.keys())
        self.n_assets = len(self.assets)
        
        # 모든 자산의 데이터 길이가 같다고 가정
        first_asset_data = data_dict[self.assets[0]]
        
        super().__init__(
            data=first_asset_data,
            initial_balance=initial_balance,
            transaction_fee=transaction_fee,
            window_size=window_size,
            max_position=max_position_per_asset
        )
        
        # 자산별 포지션
        self.positions = {asset: 0.0 for asset in self.assets}
        self.entry_prices = {asset: 0.0 for asset in self.assets}
        
        # Action space: 각 자산에 대한 행동
        self.action_space = spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(self.n_assets,),
            dtype=np.float32
        )
        
        # Observation space 재정의
        total_features = sum(data.shape[1] for data in data_dict.values())
        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(window_size, total_features + self.n_assets + 2),
            dtype=np.float32
        )
    
    def reset(self) -> np.ndarray:
        """환경 리셋"""
        self.positions = {asset: 0.0 for asset in self.assets}
        self.entry_prices = {asset: 0.0 for asset in self.assets}
        return super().reset()
    
    def _get_observation(self) -> np.ndarray:
        """다중 자산 관찰 상태 반환"""
        start_idx = self.current_step - self.window_size
        end_idx = self.current_step
        
        # 모든 자산의 시장 데이터 결합
        market_data_list = []
        for asset in self.assets:
            asset_data = self.data_dict[asset][start_idx:end_idx]
            market_data_list.append(asset_data)
        
        market_data = np.concatenate(market_data_list, axis=1)
        
        # 계정 정보
        balance_normalized = self.balance / self.initial_balance
        profit_normalized = self.total_profit / self.initial_balance
        
        # 자산별 포지션 정보
        position_info = np.array([self.positions[asset] for asset in self.assets])
        
        account_info = np.concatenate([
            [balance_normalized, profit_normalized],
            position_info
        ])
        
        # 계정 정보를 각 타임스텝에 추가
        account_info_expanded = np.tile(account_info, (self.window_size, 1))
        
        # 결합
        observation = np.concatenate([market_data, account_info_expanded], axis=1)
        
        return observation.astype(np.float32)