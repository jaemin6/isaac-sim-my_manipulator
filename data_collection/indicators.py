# data_collection/indicators.py
"""
indicators.py - 기술적 지표 계산

트레이딩에 사용되는 다양한 기술적 지표를 계산합니다.
"""

import pandas as pd
import numpy as np
from typing import Optional


class TechnicalIndicators:
    """
    기술적 지표 계산 클래스
    """
    
    @staticmethod
    def add_returns(df: pd.DataFrame, periods: list = [1, 5, 15, 30]) -> pd.DataFrame:
        """
        수익률 계산
        
        Args:
            df: OHLCV DataFrame
            periods: 계산할 기간 리스트
        
        Returns:
            수익률 컬럼이 추가된 DataFrame
        """
        for period in periods:
            df[f'return_{period}'] = df['close'].pct_change(period)
        
        return df
    
    @staticmethod
    def add_sma(df: pd.DataFrame, periods: list = [7, 20, 50, 100]) -> pd.DataFrame:
        """
        단순 이동평균 (Simple Moving Average)
        
        Args:
            df: DataFrame
            periods: 계산할 기간 리스트
        
        Returns:
            SMA 컬럼이 추가된 DataFrame
        """
        for period in periods:
            df[f'sma_{period}'] = df['close'].rolling(window=period).mean()
        
        return df
    
    @staticmethod
    def add_ema(df: pd.DataFrame, periods: list = [7, 20, 50, 100]) -> pd.DataFrame:
        """
        지수 이동평균 (Exponential Moving Average)
        
        Args:
            df: DataFrame
            periods: 계산할 기간 리스트
        
        Returns:
            EMA 컬럼이 추가된 DataFrame
        """
        for period in periods:
            df[f'ema_{period}'] = df['close'].ewm(span=period, adjust=False).mean()
        
        return df
    
    @staticmethod
    def add_rsi(df: pd.DataFrame, period: int = 14) -> pd.DataFrame:
        """
        상대강도지수 (Relative Strength Index)
        
        Args:
            df: DataFrame
            period: RSI 기간
        
        Returns:
            RSI 컬럼이 추가된 DataFrame
        """
        delta = df['close'].diff()
        
        gain = (delta.where(delta > 0, 0)).rolling(window=period).mean()
        loss = (-delta.where(delta < 0, 0)).rolling(window=period).mean()
        
        rs = gain / loss
        df[f'rsi_{period}'] = 100 - (100 / (1 + rs))
        
        return df
    
    @staticmethod
    def add_macd(
        df: pd.DataFrame,
        fast: int = 12,
        slow: int = 26,
        signal: int = 9
    ) -> pd.DataFrame:
        """
        MACD (Moving Average Convergence Divergence)
        
        Args:
            df: DataFrame
            fast: 빠른 EMA 기간
            slow: 느린 EMA 기간
            signal: 시그널 기간
        
        Returns:
            MACD 컬럼이 추가된 DataFrame
        """
        ema_fast = df['close'].ewm(span=fast, adjust=False).mean()
        ema_slow = df['close'].ewm(span=slow, adjust=False).mean()
        
        df['macd'] = ema_fast - ema_slow
        df['macd_signal'] = df['macd'].ewm(span=signal, adjust=False).mean()
        df['macd_diff'] = df['macd'] - df['macd_signal']
        
        return df
    
    @staticmethod
    def add_bollinger_bands(
        df: pd.DataFrame,
        period: int = 20,
        std_dev: float = 2.0
    ) -> pd.DataFrame:
        """
        볼린저 밴드 (Bollinger Bands)
        
        Args:
            df: DataFrame
            period: 이동평균 기간
            std_dev: 표준편차 배수
        
        Returns:
            볼린저 밴드 컬럼이 추가된 DataFrame
        """
        sma = df['close'].rolling(window=period).mean()
        std = df['close'].rolling(window=period).std()
        
        df['bb_upper'] = sma + (std * std_dev)
        df['bb_middle'] = sma
        df['bb_lower'] = sma - (std * std_dev)
        df['bb_width'] = (df['bb_upper'] - df['bb_lower']) / df['bb_middle']
        
        return df
    
    @staticmethod
    def add_atr(df: pd.DataFrame, period: int = 14) -> pd.DataFrame:
        """
        Average True Range (변동성 지표)
        
        Args:
            df: DataFrame
            period: ATR 기간
        
        Returns:
            ATR 컬럼이 추가된 DataFrame
        """
        high_low = df['high'] - df['low']
        high_close = np.abs(df['high'] - df['close'].shift())
        low_close = np.abs(df['low'] - df['close'].shift())
        
        true_range = pd.concat([high_low, high_close, low_close], axis=1).max(axis=1)
        df[f'atr_{period}'] = true_range.rolling(window=period).mean()
        
        return df
    
    @staticmethod
    def add_stochastic(
        df: pd.DataFrame,
        k_period: int = 14,
        d_period: int = 3
    ) -> pd.DataFrame:
        """
        스토캐스틱 오실레이터
        
        Args:
            df: DataFrame
            k_period: %K 기간
            d_period: %D 기간
        
        Returns:
            스토캐스틱 컬럼이 추가된 DataFrame
        """
        low_min = df['low'].rolling(window=k_period).min()
        high_max = df['high'].rolling(window=k_period).max()
        
        df['stoch_k'] = 100 * (df['close'] - low_min) / (high_max - low_min)
        df['stoch_d'] = df['stoch_k'].rolling(window=d_period).mean()
        
        return df
    
    @staticmethod
    def add_obv(df: pd.DataFrame) -> pd.DataFrame:
        """
        On-Balance Volume (거래량 지표)
        
        Args:
            df: DataFrame
        
        Returns:
            OBV 컬럼이 추가된 DataFrame
        """
        obv = [0]
        
        for i in range(1, len(df)):
            if df['close'].iloc[i] > df['close'].iloc[i-1]:
                obv.append(obv[-1] + df['volume'].iloc[i])
            elif df['close'].iloc[i] < df['close'].iloc[i-1]:
                obv.append(obv[-1] - df['volume'].iloc[i])
            else:
                obv.append(obv[-1])
        
        df['obv'] = obv
        
        return df
    
    @staticmethod
    def add_vwap(df: pd.DataFrame) -> pd.DataFrame:
        """
        Volume Weighted Average Price
        
        Args:
            df: DataFrame
        
        Returns:
            VWAP 컬럼이 추가된 DataFrame
        """
        typical_price = (df['high'] + df['low'] + df['close']) / 3
        df['vwap'] = (typical_price * df['volume']).cumsum() / df['volume'].cumsum()
        
        return df
    
    @staticmethod
    def add_momentum(df: pd.DataFrame, period: int = 10) -> pd.DataFrame:
        """
        모멘텀 지표
        
        Args:
            df: DataFrame
            period: 모멘텀 기간
        
        Returns:
            모멘텀 컬럼이 추가된 DataFrame
        """
        df[f'momentum_{period}'] = df['close'].diff(period)
        
        return df
    
    @staticmethod
    def add_all_indicators(df: pd.DataFrame) -> pd.DataFrame:
        """
        모든 기술적 지표 추가 (원스톱 함수)
        
        Args:
            df: OHLCV DataFrame
        
        Returns:
            모든 지표가 추가된 DataFrame
        """
        print("\n📊 Calculating technical indicators...")
        
        # 수익률
        df = TechnicalIndicators.add_returns(df, periods=[1, 5, 15, 30])
        print("   ✓ Returns")
        
        # 이동평균
        df = TechnicalIndicators.add_sma(df, periods=[7, 20, 50])
        df = TechnicalIndicators.add_ema(df, periods=[7, 20, 50])
        print("   ✓ Moving Averages (SMA, EMA)")
        
        # RSI
        df = TechnicalIndicators.add_rsi(df, period=14)
        print("   ✓ RSI")
        
        # MACD
        df = TechnicalIndicators.add_macd(df)
        print("   ✓ MACD")
        
        # 볼린저 밴드
        df = TechnicalIndicators.add_bollinger_bands(df, period=20)
        print("   ✓ Bollinger Bands")
        
        # ATR
        df = TechnicalIndicators.add_atr(df, period=14)
        print("   ✓ ATR")
        
        # 스토캐스틱
        df = TechnicalIndicators.add_stochastic(df, k_period=14, d_period=3)
        print("   ✓ Stochastic")
        
        # OBV
        df = TechnicalIndicators.add_obv(df)
        print("   ✓ OBV")
        
        # VWAP
        df = TechnicalIndicators.add_vwap(df)
        print("   ✓ VWAP")
        
        # 모멘텀
        df = TechnicalIndicators.add_momentum(df, period=10)
        print("   ✓ Momentum")
        
        # 결측치 제거 (초기 계산 불가능한 값들)
        initial_len = len(df)
        df = df.dropna().reset_index(drop=True)
        removed = initial_len - len(df)
        
        if removed > 0:
            print(f"\n   Removed {removed} rows with NaN values")
        
        print(f"✅ Technical indicators complete: {len(df)} rows")
        
        return df


def test_indicators():
    """지표 계산 테스트"""
    print("\n" + "="*60)
    print("🧪 Testing Technical Indicators")
    print("="*60)
    
    # 샘플 데이터 생성
    dates = pd.date_range(start='2024-01-01', periods=200, freq='1H')
    np.random.seed(42)
    
    df = pd.DataFrame({
        'timestamp': dates,
        'open': 50000 + np.cumsum(np.random.randn(200) * 100),
        'high': 50000 + np.cumsum(np.random.randn(200) * 100) + 100,
        'low': 50000 + np.cumsum(np.random.randn(200) * 100) - 100,
        'close': 50000 + np.cumsum(np.random.randn(200) * 100),
        'volume': np.random.randint(100, 1000, 200)
    })
    
    # 지표 추가
    df_with_indicators = TechnicalIndicators.add_all_indicators(df)
    
    print("\n📊 Columns:")
    print(df_with_indicators.columns.tolist())
    
    print("\n📈 Sample data:")
    print(df_with_indicators.tail())
    
    return df_with_indicators


if __name__ == '__main__':
    test_indicators()