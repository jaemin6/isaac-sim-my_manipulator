# data_collection/collect_data.py
"""
collect_data.py - 암호화폐 데이터 수집 파이프라인

Binance에서 데이터를 수집하고 기술적 지표를 계산한 뒤,
정규화하여 학습/검증/테스트 데이터로 분할 저장합니다.
"""

import argparse
from datetime import datetime
import os

from collectors import BinanceCollector, DataValidator
from indicators import TechnicalIndicators
from preprocessor import DataPreprocessor, DataSaver


def collect_and_process_data(
    symbol: str = 'BTC/USDT',
    timeframe: str = '1m',
    days: int = 30,
    train_ratio: float = 0.7,
    val_ratio: float = 0.15,
    test_ratio: float = 0.15,
    scaler_type: str = 'standard',
    output_dir: str = 'data/processed'
):
    """
    데이터 수집 및 전처리 파이프라인
    
    Args:
        symbol: 거래 쌍
        timeframe: 시간봉
        days: 수집할 일 수
        train_ratio: 학습 데이터 비율
        val_ratio: 검증 데이터 비율
        test_ratio: 테스트 데이터 비율
        scaler_type: 정규화 방법
        output_dir: 출력 디렉토리
    """
    print("\n" + "="*60)
    print("🚀 암호화폐 데이터 수집 파이프라인")
    print("="*60)
    print(f"Symbol: {symbol}")
    print(f"Timeframe: {timeframe}")
    print(f"Days: {days}")
    print(f"Output: {output_dir}")
    print("="*60)
    
    # 1. 데이터 수집
    print("\n[Step 1/5] 📡 Collecting data from Binance...")
    collector = BinanceCollector()
    df = collector.fetch_ohlcv(
        symbol=symbol,
        timeframe=timeframe,
        days=days
    )
    
    # 원본 데이터 저장
    DataSaver.save_raw_data(df, filename=f'{symbol.replace("/", "_")}_{timeframe}.csv')
    
    # 2. 데이터 검증
    print("\n[Step 2/5] 🔍 Validating data...")
    validator = DataValidator()
    df_clean = validator.validate_ohlcv(df)
    
    # 타임스탬프 보간 (선택사항)
    if timeframe in ['1m', '5m', '15m', '1h']:
        df_clean = validator.fill_missing_timestamps(df_clean, timeframe)
    
    # 3. 기술적 지표 계산
    print("\n[Step 3/5] 📊 Calculating technical indicators...")
    df_with_indicators = TechnicalIndicators.add_all_indicators(df_clean)
    
    print(f"\n   Total features: {len(df_with_indicators.columns)}")
    print(f"   Feature columns: {df_with_indicators.columns.tolist()}")
    
    # 4. 데이터 분할 및 정규화
    print("\n[Step 4/5] 🔧 Splitting and scaling data...")
    preprocessor = DataPreprocessor(scaler_type=scaler_type)
    
    # 분할
    train_df, val_df, test_df = preprocessor.split_data(
        df_with_indicators,
        train_ratio=train_ratio,
        val_ratio=val_ratio,
        test_ratio=test_ratio
    )
    
    # 정규화
    train_scaled = preprocessor.fit_transform(train_df, exclude_columns=['timestamp'])
    val_scaled = preprocessor.transform(val_df)
    test_scaled = preprocessor.transform(test_df)
    
    # Scaler 저장
    scaler_path = os.path.join(output_dir, 'scaler_params.json')
    preprocessor.save_scaler(scaler_path)
    
    # 5. 데이터 저장
    print("\n[Step 5/5] 💾 Saving processed data...")
    DataSaver.save_to_csv(
        train_scaled,
        val_scaled,
        test_scaled,
        output_dir=output_dir
    )
    
    # 메타데이터 저장
    metadata = {
        'collection_date': datetime.now().isoformat(),
        'symbol': symbol,
        'timeframe': timeframe,
        'days': days,
        'total_samples': len(df_with_indicators),
        'train_samples': len(train_scaled),
        'val_samples': len(val_scaled),
        'test_samples': len(test_scaled),
        'features': df_with_indicators.columns.tolist(),
        'scaler_type': scaler_type,
        'train_ratio': train_ratio,
        'val_ratio': val_ratio,
        'test_ratio': test_ratio
    }
    DataSaver.save_metadata(metadata, output_dir=output_dir)
    
    print("\n" + "="*60)
    print("✅ 데이터 수집 완료!")
    print("="*60)
    print(f"\n📊 Summary:")
    print(f"   Total samples: {len(df_with_indicators)}")
    print(f"   Features: {len(df_with_indicators.columns)}")
    print(f"   Train: {len(train_scaled)} samples")
    print(f"   Val: {len(val_scaled)} samples")
    print(f"   Test: {len(test_scaled)} samples")
    print(f"\n📁 Output directory: {output_dir}")
    print(f"   - train_scaled.csv")
    print(f"   - val_scaled.csv")
    print(f"   - test_scaled.csv")
    print(f"   - scaler_params.json")
    print(f"   - metadata.json")
    print("\n🚀 Ready for Phase 4 training!")
    print("   Run: python train_rl.py")
    print("="*60)


def collect_multiple_symbols(
    symbols: list = ['BTC/USDT', 'ETH/USDT'],
    timeframe: str = '1m',
    days: int = 30,
    output_dir: str = 'data/processed/multi'
):
    """
    여러 심볼의 데이터 수집
    
    Args:
        symbols: 심볼 리스트
        timeframe: 시간봉
        days: 수집할 일 수
        output_dir: 출력 디렉토리
    """
    print("\n" + "="*60)
    print("🚀 다중 심볼 데이터 수집")
    print("="*60)
    
    collector = BinanceCollector()
    
    # 데이터 수집
    data_dict = collector.fetch_multiple_symbols(symbols, timeframe, days)
    
    # 각 심볼별 처리
    for symbol, df in data_dict.items():
        print(f"\n{'='*60}")
        print(f"Processing {symbol}...")
        print(f"{'='*60}")
        
        symbol_output = os.path.join(output_dir, symbol.replace('/', '_'))
        
        collect_and_process_data(
            symbol=symbol,
            timeframe=timeframe,
            days=days,
            output_dir=symbol_output
        )


def main():
    parser = argparse.ArgumentParser(description='암호화폐 데이터 수집 및 전처리')
    
    parser.add_argument('--symbol', type=str, default='BTC/USDT',
                       help='거래 쌍 (예: BTC/USDT)')
    
    parser.add_argument('--timeframe', type=str, default='1m',
                       choices=['1m', '5m', '15m', '1h', '4h', '1d'],
                       help='시간봉')
    
    parser.add_argument('--days', type=int, default=30,
                       help='수집할 일 수')
    
    parser.add_argument('--train-ratio', type=float, default=0.7,
                       help='학습 데이터 비율')
    
    parser.add_argument('--val-ratio', type=float, default=0.15,
                       help='검증 데이터 비율')
    
    parser.add_argument('--test-ratio', type=float, default=0.15,
                       help='테스트 데이터 비율')
    
    parser.add_argument('--scaler', type=str, default='standard',
                       choices=['standard', 'minmax', 'robust'],
                       help='정규화 방법')
    
    parser.add_argument('--output', type=str, default='data/processed',
                       help='출력 디렉토리')
    
    parser.add_argument('--multi', action='store_true',
                       help='여러 심볼 수집 (BTC, ETH)')
    
    args = parser.parse_args()
    
    # 다중 심볼 수집
    if args.multi:
        symbols = ['BTC/USDT', 'ETH/USDT', 'BNB/USDT']
        collect_multiple_symbols(
            symbols=symbols,
            timeframe=args.timeframe,
            days=args.days,
            output_dir=args.output
        )
    else:
        # 단일 심볼 수집
        collect_and_process_data(
            symbol=args.symbol,
            timeframe=args.timeframe,
            days=args.days,
            train_ratio=args.train_ratio,
            val_ratio=args.val_ratio,
            test_ratio=args.test_ratio,
            scaler_type=args.scaler,
            output_dir=args.output
        )


if __name__ == '__main__':
    main()