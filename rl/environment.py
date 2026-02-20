# rl/environment.py (V3 - 과도한 거래 방지)
"""
트레이딩 환경 V3 - 좋은 거래 장려

핵심 변경:
1. 최소 홀딩 시간 도입 (5 스텝)
2. 홀딩 시간에 비례한 보상
3. 빈번한 거래 페널티
"""

import numpy as np
import torch
from typing import Dict, Tuple, Optional
import gym
from gym import spaces


class TradingEnvironment(gym.Env):
    """
    트레이딩 환경 V3 (좋은 거래 장려)
    
    설계 철학:
    - 좋은 거래 = 큰 보상
    - 나쁜 거래 = 페널티
    - 빈번한 거래 = 페널티
    - 홀딩 인내 = 보상
    """
    
    def __init__(
        self,
        data: np.ndarray,
        initial_balance: float = 10000.0,
        transaction_fee: float = 0.001,
        window_size: int = 60,
        max_position: float = 1.0,
        verbose: bool = False,
        # 🆕 거래 제어 파라미터
        min_hold_steps: int = 5,  # 최소 홀딩 시간
        position_change_threshold: float = 0.005,  # 포지션 변경 임계값
    ):
        super(TradingEnvironment, self).__init__()
        
        self.data = data.copy()
        self.verbose = verbose
        self.min_hold_steps = min_hold_steps
        self.position_change_threshold = position_change_threshold
        
        # 가격 복원
        self._restore_price_if_normalized()
        
        self.initial_balance = initial_balance
        self.transaction_fee = transaction_fee
        self.window_size = window_size
        self.max_position = max_position
        
        # 환경 상태
        self.current_step = 0
        self.balance = initial_balance
        self.position = 0.0
        self.entry_price = 0.0
        self.total_profit = 0.0
        
        # 🆕 홀딩 시간 추적
        self.hold_duration = 0
        self.last_trade_step = 0
        
        # 거래 통계
        self.num_trades = 0
        self.winning_trades = 0
        self.losing_trades = 0
        self.trade_history = []
        
        # 에피소드 통계
        self.episode_steps = 0
        self.max_portfolio_value = initial_balance
        
        # Action/Observation space
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(1,), dtype=np.float32
        )
        
        n_features = data.shape[1]
        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(window_size, n_features + 5),  # +5
            dtype=np.float32
        )
    
    def _restore_price_if_normalized(self):
        """정규화된 가격 복원"""
        price_mean = self.data[:, 0].mean()
        price_min = self.data[:, 0].min()
        price_max = self.data[:, 0].max()
        
        is_normalized = (abs(price_mean) < 10) and (price_min > -10) and (price_max < 10)
        
        if is_normalized:
            if self.verbose:
                print("⚠️  Normalized price detected! Restoring...")
            
            original_range = price_max - price_min
            target_min = 30000
            target_range = 30000
            
            if original_range > 0:
                scale = target_range / original_range
                self.data[:, 0] = (self.data[:, 0] - price_min) * scale + target_min
            else:
                self.data[:, 0] = 45000
            
            if self.verbose:
                print(f"✅ Price restored: range=[${self.data[:, 0].min():.2f}, ${self.data[:, 0].max():.2f}]")
    
    def reset(self) -> np.ndarray:
        """환경 리셋"""
        self.current_step = self.window_size
        self.balance = self.initial_balance
        self.position = 0.0
        self.entry_price = 0.0
        self.total_profit = 0.0
        
        self.hold_duration = 0
        self.last_trade_step = 0
        
        self.num_trades = 0
        self.winning_trades = 0
        self.losing_trades = 0
        self.trade_history = []
        
        self.episode_steps = 0
        self.max_portfolio_value = self.initial_balance
        
        return self._get_observation()
    
    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, Dict]:
        """환경 스텝"""
        action = np.clip(action[0], -1.0, 1.0)
        current_price = self._get_current_price()
        
        # 🔥 V3 보상 계산
        reward = self._calculate_v3_reward(action, current_price)
        
        self.current_step += 1
        self.episode_steps += 1
        self.hold_duration += 1
        
        # 종료 조건
        done = False
        
        if self.current_step >= len(self.data) - 1:
            done = True
        
        if self.balance <= 0:
            done = True
            reward -= 100.0
        
        if self.episode_steps >= 2000:
            done = True
        
        observation = self._get_observation()
        
        portfolio_value = self.get_portfolio_value()
        if portfolio_value > self.max_portfolio_value:
            self.max_portfolio_value = portfolio_value
        
        info = {
            'balance': self.balance,
            'position': self.position,
            'total_profit': self.total_profit,
            'current_price': current_price,
            'num_trades': self.num_trades,
            'win_rate': self.winning_trades / max(self.num_trades, 1),
            'portfolio_value': portfolio_value,
            'hold_duration': self.hold_duration,
        }
        
        return observation, reward, done, info
    
    def _calculate_v3_reward(self, action: float, current_price: float) -> float:
        """
        🔥 V3 보상 함수 - 좋은 거래 장려
        
        핵심:
        1. 최소 홀딩 시간 강제
        2. 수익 거래에만 큰 보상
        3. 손실 거래는 강한 페널티
        4. 홀딩 시간에 비례한 보너스
        """
        target_position = action * self.max_position
        position_change = abs(target_position - self.position)
        
        reward = 0.0
        
        # ============================================
        # [1] 거래 시도 (포지션 변경)
        # ============================================
        if position_change > self.position_change_threshold:
            
            # 🆕 최소 홀딩 시간 체크
            steps_since_trade = self.current_step - self.last_trade_step
            
            if abs(self.position) > 0.05 and steps_since_trade < self.min_hold_steps:
                # 너무 빨리 거래 시도 → 강한 페널티
                return -5.0
            
            # (A) 기존 포지션 청산
            if abs(self.position) > 0.05:
                # 수익률 계산
                if self.position > 0:
                    price_change_pct = (current_price - self.entry_price) / self.entry_price
                else:
                    price_change_pct = (self.entry_price - current_price) / self.entry_price
                
                # 실현 손익
                position_size = abs(self.position) * self.initial_balance
                gross_pnl = price_change_pct * position_size
                fee = position_size * self.transaction_fee
                net_pnl = gross_pnl - fee
                
                # 잔고 업데이트
                self.balance += net_pnl
                self.total_profit += net_pnl
                
                # 🔥 핵심: 수익/손실에 따른 차별화된 보상
                profit_ratio = net_pnl / self.initial_balance
                
                if net_pnl > 0:
                    # 🔥 수익 거래 = 매우 큰 보상
                    base_reward = profit_ratio * 200  # 2배 증가
                    
                    # 🔥 홀딩 시간 보너스 (오래 들고 있을수록 보상)
                    hold_bonus = min(self.hold_duration / 20.0, 2.0)  # 최대 2배
                    reward += base_reward * (1 + hold_bonus)
                    
                    self.winning_trades += 1
                else:
                    # 🔥 손실 거래 = 강한 페널티
                    reward += profit_ratio * 200  # 2배 증가
                    
                    # 🔥 빠른 손절 보너스 (손실은 빨리 끊을수록 좋음)
                    if self.hold_duration < 10:
                        reward += 1.0  # 빠른 손절 보너스
                    
                    self.losing_trades += 1
                
                self.num_trades += 1
                self.trade_history.append({
                    'step': self.current_step,
                    'type': 'close',
                    'position': self.position,
                    'entry_price': self.entry_price,
                    'exit_price': current_price,
                    'pnl': net_pnl,
                    'hold_duration': self.hold_duration,
                    'balance': self.balance
                })
                
                # 홀딩 시간 리셋
                self.hold_duration = 0
            
            # (B) 새 포지션 진입
            if abs(target_position) > 0.005:
                entry_size = abs(target_position) * self.initial_balance
                entry_fee = entry_size * self.transaction_fee
                self.balance -= entry_fee
                
                self.position = target_position
                self.entry_price = current_price
                self.last_trade_step = self.current_step
                
                # 진입 비용
                reward -= (entry_fee / self.initial_balance) * 10
                
                self.trade_history.append({
                    'step': self.current_step,
                    'type': 'open',
                    'position': self.position,
                    'entry_price': current_price,
                    'balance': self.balance
                })
            else:
                # 완전 청산
                self.position = 0.0
                self.entry_price = 0.0
        
        # ============================================
        # [2] 홀딩 (포지션 유지)
        # ============================================
        else:
            if abs(self.position) > 0.05:
                # 미실현 손익
                if self.position > 0:
                    unrealized_pnl_pct = (current_price - self.entry_price) / self.entry_price
                else:
                    unrealized_pnl_pct = (self.entry_price - current_price) / self.entry_price
                
                # 🔥 홀딩 보상 (수익 중일 때만)
                if unrealized_pnl_pct > 0:
                    # 수익 중 = 유지 장려
                    reward += unrealized_pnl_pct * 2
                    
                    # 오래 들고 있을수록 추가 보너스
                    if self.hold_duration > 20:
                        reward += 0.5
                else:
                    # 손실 중 = 작은 페널티
                    reward += unrealized_pnl_pct * 5
        
        # ============================================
        # [3] 자산 상태 체크
        # ============================================
        portfolio_value = self.get_portfolio_value()
        
        if portfolio_value < self.initial_balance * 0.5:
            reward -= 5.0
        
        if portfolio_value < self.initial_balance * 0.3:
            reward -= 20.0
        
        return reward
    
    def _get_observation(self) -> np.ndarray:
        """관찰 상태 반환"""
        start_idx = self.current_step - self.window_size
        end_idx = self.current_step
        market_data = self.data[start_idx:end_idx]
        
        # 계정 정보
        balance_normalized = self.balance / self.initial_balance
        position_info = self.position
        profit_normalized = self.total_profit / self.initial_balance
        
        # 미실현 손익
        if abs(self.position) > 0.05:
            current_price = self._get_current_price()
            if self.position > 0:
                unrealized_pnl = (current_price - self.entry_price) / self.entry_price
            else:
                unrealized_pnl = (self.entry_price - current_price) / self.entry_price
        else:
            unrealized_pnl = 0.0
        
        # 🆕 홀딩 시간 정보
        hold_duration_normalized = self.hold_duration / 50.0  # 50 스텝으로 정규화
        
        account_info = np.array([
            balance_normalized,
            position_info,
            profit_normalized,
            unrealized_pnl,
            hold_duration_normalized
        ])
        
        account_info_expanded = np.tile(account_info, (self.window_size, 1))
        observation = np.concatenate([market_data, account_info_expanded], axis=1)
        
        return observation.astype(np.float32)
    
    def _get_current_price(self) -> float:
        return self.data[self.current_step, 0]
    
    def get_portfolio_value(self) -> float:
        """포트폴리오 가치"""
        current_price = self._get_current_price()
        
        if abs(self.position) > 0.05:
            position_size = abs(self.position) * self.initial_balance
            
            if self.position > 0:
                unrealized_pnl = (current_price - self.entry_price) / self.entry_price * position_size
            else:
                unrealized_pnl = (self.entry_price - current_price) / self.entry_price * position_size
        else:
            unrealized_pnl = 0
        
        return self.balance + unrealized_pnl
    
    def render(self, mode='human'):
        """환경 시각화"""
        if mode == 'human':
            profit_pct = (self.total_profit / self.initial_balance) * 100
            win_rate = self.winning_trades / max(self.num_trades, 1) * 100
            portfolio_value = self.get_portfolio_value()
            
            print(f"Step: {self.current_step}")
            print(f"Balance: ${self.balance:.2f}")
            print(f"Portfolio Value: ${portfolio_value:.2f}")
            print(f"Position: {self.position:.2f}")
            print(f"Total Profit: ${self.total_profit:.2f} ({profit_pct:.2f}%)")
            print(f"Trades: {self.num_trades} (Win Rate: {win_rate:.1f}%)")
            print(f"Hold Duration: {self.hold_duration}")
            print("-" * 50)


# 하위 호환성
class CurriculumTradingEnv(TradingEnvironment):
    """커리큘럼 학습 환경 (V3에서는 필요 없지만 호환성 유지)"""
    pass


class MultiAssetTradingEnvironment(TradingEnvironment):
    """다중 자산 환경"""
    pass