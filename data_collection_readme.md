# 데이터 수집 가이드 

Phase 4 강화학습을 위한 암호화폐 데이터 수집 및 전처리

## 빠른 시작

### 1. 필수 라이브러리 설치

```bash
pip install ccxt pandas numpy scikit-learn
```

### 2. 기본 데이터 수집

```bash
# BTC 30일 데이터 수집 (1분봉)
python data_collection/collect_data.py --symbol BTC/USDT --timeframe 1m --days 30
```

## 사용 예시

### 기본 수집
```bash
# BTC 1분봉 30일
python data_collection/collect_data.py
```

### 다른 심볼
```bash
# 이더리움
python data_collection/collect_data.py --symbol ETH/USDT

# 바이낸스 코인
python data_collection/collect_data.py --symbol BNB/USDT
```

### 다른 시간봉
```bash
# 1시간봉
python data_collection/collect_data.py --timeframe 1h --days 90

# 15분봉
python data_collection/collect_data.py --timeframe 15m --days 60

# 일봉
python data_collection/collect_data.py --timeframe 1d --days 365
```

### 다중 심볼 수집
```bash
# BTC, ETH, BNB 동시 수집
python data_collection/collect_data.py --multi --timeframe 1h --days 30
```

### 데이터 분할 비율 조정
```bash
# Train 80%, Val 10%, Test 10%
python data_collection/collect_data.py --train-ratio 0.8 --val-ratio 0.1 --test-ratio 0.1
```

### 정규화 방법 선택
```bash
# MinMax 스케일러
python data_collection/collect_data.py --scaler minmax

# Robust 스케일러 (이상치에 강함)
python data_collection/collect_data.py --scaler robust
```

## 수집되는 데이터

### OHLCV 기본 데이터
- Open (시가)
- High (고가)
- Low (저가)
- Close (종가)
- Volume (거래량)

### 기술적 지표 (자동 계산)
- **이동평균**: SMA(7,20,50), EMA(7,20,50)
- **모멘텀**: RSI(14), MACD, Stochastic
- **변동성**: Bollinger Bands, ATR
- **거래량**: OBV, VWAP
- **수익률**: 1분, 5분, 15분, 30분 수익률

## 출력 파일

```
data/
├── raw/
│   └── BTC_USDT_1m.csv          # 원본 데이터
└── processed/
    ├── train_scaled.csv         # 학습 데이터          O
    ├── val_scaled.csv           # 검증 데이터          O
    ├── test_scaled.csv          # 테스트 데이터        O
    ├── scaler_params.json       # 정규화 파라미터
    └── metadata.json            # 메타 정보
```

## 커맨드 라인 옵션

| 옵션 | 설명 | 기본값 |
|------|------|--------|
| `--symbol` | 거래 쌍 | BTC/USDT |
| `--timeframe` | 시간봉 | 1m |
| `--days` | 수집 기간 | 30 |
| `--train-ratio` | 학습 비율 | 0.7 |
| `--val-ratio` | 검증 비율 | 0.15 |
| `--test-ratio` | 테스트 비율 | 0.15 |
| `--scaler` | 정규화 방법 | standard |
| `--output` | 출력 경로 | data/processed |
| `--multi` | 다중 심볼 수집 | False |

## 추천 설정

### 빠른 테스트용
```bash
# 3일, 1시간봉 (빠름)
python data_collection/collect_data.py --timeframe 1h --days 3
```

### 실전 학습용
```bash
# 30일, 1분봉 (품질 좋음)
python data_collection/collect_data.py --timeframe 1m --days 30
```

### 장기 백테스팅용
```bash
# 180일, 1시간봉 (긴 기간)
python data_collection/collect_data.py --timeframe 1h --days 180
```

## 주의사항

1. **API 제한**: Binance API는 요청 제한이 있습니다. 긴 기간 수집 시 시간이 걸립니다.
2. **인터넷 연결**: 안정적인 인터넷 연결이 필요합니다.
3. **저장 공간**: 1분봉 30일 데이터는 약 40,000개 캔들 (수 MB)
4. **데이터 품질**: 주말/공휴일에도 24시간 거래되는 암호화폐 특성상 데이터 연속성 좋음

## 트러블슈팅

### "ccxt 모듈을 찾을 수 없습니다"
```bash
pip install ccxt
```

### "API 요청 제한 초과"
- 잠시 대기 후 다시 시도
- `--days` 값을 줄여서 시도

### "데이터가 너무 적습니다"
- `--days` 값을 늘리기
- 더 긴 시간봉 사용 (1m → 5m → 15m → 1h)

## 다음 단계

데이터 수집 완료 후:

```bash
# Phase 4 학습 시작
python train_rl.py --config default
```

---
