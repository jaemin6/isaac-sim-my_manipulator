# data_collection.py
"""
collectors.py - 암호화폐 데이터 수집

Binance API를 사용하여 실제 암호화폐 가격 데이터를 수집합니다.
"""

import ccxt
import pandas as pd
import numpy as np
from datetime import datetime, timedelta
from typing import Optional, List
import time


class BinanceCollector:
    """
    Binance 거래소에서 암호화폐 데이터 수집
    """
    
    def __init__(self):
        """Binance 거래소 연결"""
        self.exchange = ccxt.binance({
            'enableRateLimit': True,  # API 요청 제한 준수
            'options': {
                'defaultType': 'spot',  # 현물 거래
            }
        })
        print("✅ Binance API connected")
    
    def fetch_ohlcv(
        self,
        symbol: str = 'BTC/USDT',
        timeframe: str = '1m',
        days: int = 30,
        limit: int = 1000
    ) -> pd.DataFrame:
        """
        OHLCV 데이터 수집 (Open, High, Low, Close, Volume)
        
        Args:
            symbol: 거래 쌍 (예: 'BTC/USDT')
            timeframe: 시간봉 ('1m', '5m', '15m', '1h', '1d')
            days: 수집할 일 수
            limit: 한 번에 가져올 캔들 개수
        
        Returns:
            DataFrame with OHLCV data
        """
        print(f"\n📊 Fetching {symbol} {timeframe} data for {days} days...")
        
        # 시작 시간 계산
        since = self.exchange.parse8601(
            (datetime.utcnow() - timedelta(days=days)).isoformat()
        )
        
        all_ohlcv = []
        
        while True:
            try:
                # 데이터 요청
                ohlcv = self.exchange.fetch_ohlcv(
                    symbol=symbol,
                    timeframe=timeframe,
                    since=since,
                    limit=limit
                )
                
                if not ohlcv:
                    break
                
                all_ohlcv.extend(ohlcv)
                
                # 다음 요청을 위한 시작 시간 업데이트
                since = ohlcv[-1][0] + 1
                
                print(f"   Fetched {len(ohlcv)} candles... (Total: {len(all_ohlcv)})")
                
                # 현재 시간에 도달하면 종료
                if ohlcv[-1][0] >= self.exchange.milliseconds():
                    break
                
                # Rate limit 준수
                time.sleep(self.exchange.rateLimit / 1000)
                
            except Exception as e:
                print(f"⚠️  Error fetching data: {e}")
                break
        
        # DataFrame 생성
        df = pd.DataFrame(
            all_ohlcv,
            columns=['timestamp', 'open', 'high', 'low', 'close', 'volume']
        )
        
        # Timestamp 변환
        df['timestamp'] = pd.to_datetime(df['timestamp'], unit='ms')
        
        # 중복 제거
        df = df.drop_duplicates(subset=['timestamp']).reset_index(drop=True)
        
        print(f"✅ Collected {len(df)} candles")
        print(f"   Period: {df['timestamp'].min()} to {df['timestamp'].max()}")
        
        return df
    
    def fetch_multiple_symbols(
        self,
        symbols: List[str],
        timeframe: str = '1m',
        days: int = 30
    ) -> dict:
        """
        여러 심볼의 데이터 동시 수집
        
        Args:
            symbols: 심볼 리스트 (예: ['BTC/USDT', 'ETH/USDT'])
            timeframe: 시간봉
            days: 수집할 일 수
        
        Returns:
            {symbol: DataFrame} 딕셔너리
        """
        data_dict = {}
        
        for symbol in symbols:
            print(f"\n📈 Collecting {symbol}...")
            df = self.fetch_ohlcv(symbol, timeframe, days)
            data_dict[symbol] = df
            time.sleep(1)  # 심볼 간 대기
        
        return data_dict
    
    def get_available_symbols(self, quote: str = 'USDT') -> List[str]:
        """
        거래 가능한 심볼 목록 조회
        
        Args:
            quote: 기준 통화 (예: 'USDT', 'BTC')
        
        Returns:
            심볼 리스트
        """
        try:
            markets = self.exchange.load_markets()
            symbols = [
                symbol for symbol in markets.keys()
                if symbol.endswith(f'/{quote}') and markets[symbol]['active']
            ]
            return sorted(symbols)
        except Exception as e:
            print(f"⚠️  Error loading markets: {e}")
            return []


class DataValidator:
    """
    수집한 데이터 검증 및 정제
    """
    
    @staticmethod
    def validate_ohlcv(df: pd.DataFrame) -> pd.DataFrame:
        """
        OHLCV 데이터 검증 및 정제
        
        Args:
            df: OHLCV DataFrame
        
        Returns:
            정제된 DataFrame
        """
        print("\n🔍 Validating data...")
        
        initial_len = len(df)
        
        # 1. 결측치 확인
        missing = df.isnull().sum()
        if missing.any():
            print(f"   Found missing values:")
            print(missing[missing > 0])
            df = df.dropna()
        
        # 2. 가격 이상치 확인 (0 또는 음수)
        price_cols = ['open', 'high', 'low', 'close']
        for col in price_cols:
            invalid = (df[col] <= 0).sum()
            if invalid > 0:
                print(f"   Found {invalid} invalid prices in {col}")
                df = df[df[col] > 0]
        
        # 3. High >= Low 확인
        invalid_hl = (df['high'] < df['low']).sum()
        if invalid_hl > 0:
            print(f"   Found {invalid_hl} candles with high < low")
            df = df[df['high'] >= df['low']]
        
        # 4. OHLC 관계 확인 (high가 최고가, low가 최저가)
        invalid_o = ((df['open'] > df['high']) | (df['open'] < df['low'])).sum()
        invalid_c = ((df['close'] > df['high']) | (df['close'] < df['low'])).sum()
        
        if invalid_o > 0 or invalid_c > 0:
            print(f"   Found {invalid_o + invalid_c} invalid OHLC relationships")
            df = df[
                (df['open'] >= df['low']) & (df['open'] <= df['high']) &
                (df['close'] >= df['low']) & (df['close'] <= df['high'])
            ]
        
        # 5. 거래량 확인
        invalid_vol = (df['volume'] < 0).sum()
        if invalid_vol > 0:
            print(f"   Found {invalid_vol} negative volumes")
            df = df[df['volume'] >= 0]
        
        # 6. 중복 제거
        df = df.drop_duplicates(subset=['timestamp']).reset_index(drop=True)
        
        # 7. 시간순 정렬
        df = df.sort_values('timestamp').reset_index(drop=True)
        
        final_len = len(df)
        removed = initial_len - final_len
        
        if removed > 0:
            print(f"   Removed {removed} invalid rows ({removed/initial_len*100:.2f}%)")
        
        print(f"✅ Validation complete: {final_len} valid candles")
        
        return df
    
    @staticmethod
    def fill_missing_timestamps(
        df: pd.DataFrame,
        timeframe: str = '1m'
    ) -> pd.DataFrame:
        """
        누락된 타임스탬프 보간
        
        Args:
            df: DataFrame
            timeframe: 시간봉
        
        Returns:
            보간된 DataFrame
        """
        print("\n🔧 Filling missing timestamps...")
        
        # 시간 간격 매핑
        freq_map = {
            '1m': '1min',
            '5m': '5min',
            '15m': '15min',
            '1h': '1H',
            '1d': '1D'
        }
        
        freq = freq_map.get(timeframe, '1min')
        
        # 전체 시간 범위 생성
        full_range = pd.date_range(
            start=df['timestamp'].min(),
            end=df['timestamp'].max(),
            freq=freq
        )
        
        initial_len = len(df)
        
        # 리인덱싱
        df = df.set_index('timestamp').reindex(full_range)
        
        # 결측치 보간 (forward fill)
        df = df.fillna(method='ffill')
        
        # 인덱스를 다시 컬럼으로
        df = df.reset_index().rename(columns={'index': 'timestamp'})
        
        added = len(df) - initial_len
        if added > 0:
            print(f"   Added {added} missing timestamps")
        
        return df


def quick_test():
    """빠른 테스트 함수"""
    print("\n" + "="*60)
    print("🧪 Quick Test: Binance Data Collection")
    print("="*60)
    
    collector = BinanceCollector()
    
    # BTC 1시간 데이터 수집 (3일)
    df = collector.fetch_ohlcv(
        symbol='BTC/USDT',
        timeframe='1h',
        days=3
    )
    
    # 데이터 검증
    validator = DataValidator()
    df_clean = validator.validate_ohlcv(df)
    
    # 통계 출력
    print("\n📊 Data Statistics:")
    print(df_clean.describe())
    
    return df_clean


if __name__ == '__main__':
    # 테스트 실행
    df = quick_test()