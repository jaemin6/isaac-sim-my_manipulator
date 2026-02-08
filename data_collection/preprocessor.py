# data_collection/preprocessor.py
"""
preprocessor.py - 데이터 전처리

데이터 정규화, 분할, 저장을 담당합니다.
"""

import pandas as pd
import numpy as np
from sklearn.preprocessing import StandardScaler, MinMaxScaler, RobustScaler
from typing import Tuple, Optional
import os
import json


class DataPreprocessor:
    """
    데이터 전처리 클래스
    """
    
    def __init__(self, scaler_type: str = 'standard'):
        """
        Args:
            scaler_type: 'standard', 'minmax', 'robust'
        """
        self.scaler_type = scaler_type
        self.scaler = None
        self.feature_columns = None
        
        # Scaler 선택
        if scaler_type == 'standard':
            self.scaler = StandardScaler()
        elif scaler_type == 'minmax':
            self.scaler = MinMaxScaler()
        elif scaler_type == 'robust':
            self.scaler = RobustScaler()
        else:
            raise ValueError(f"Unknown scaler type: {scaler_type}")
        
        print(f"✅ Preprocessor initialized with {scaler_type} scaler")
    
    def split_data(
        self,
        df: pd.DataFrame,
        train_ratio: float = 0.7,
        val_ratio: float = 0.15,
        test_ratio: float = 0.15
    ) -> Tuple[pd.DataFrame, pd.DataFrame, pd.DataFrame]:
        """
        데이터를 Train/Val/Test로 분할
        
        Args:
            df: DataFrame
            train_ratio: 학습 데이터 비율
            val_ratio: 검증 데이터 비율
            test_ratio: 테스트 데이터 비율
        
        Returns:
            train_df, val_df, test_df
        """
        assert abs(train_ratio + val_ratio + test_ratio - 1.0) < 1e-6, \
            "Ratios must sum to 1.0"
        
        n = len(df)
        train_end = int(n * train_ratio)
        val_end = int(n * (train_ratio + val_ratio))
        
        train_df = df[:train_end].copy()
        val_df = df[train_end:val_end].copy()
        test_df = df[val_end:].copy()
        
        print(f"\n📊 Data split:")
        print(f"   Train: {len(train_df)} samples ({len(train_df)/n*100:.1f}%)")
        print(f"   Val:   {len(val_df)} samples ({len(val_df)/n*100:.1f}%)")
        print(f"   Test:  {len(test_df)} samples ({len(test_df)/n*100:.1f}%)")
        
        return train_df, val_df, test_df
    
    def fit_transform(
        self,
        train_df: pd.DataFrame,
        exclude_columns: list = ['timestamp']
    ) -> pd.DataFrame:
        """
        학습 데이터로 Scaler를 학습하고 변환
        
        Args:
            train_df: 학습 DataFrame
            exclude_columns: 정규화에서 제외할 컬럼
        
        Returns:
            정규화된 DataFrame
        """
        # 정규화할 컬럼 선택
        self.feature_columns = [
            col for col in train_df.columns
            if col not in exclude_columns
        ]
        
        print(f"\n🔧 Fitting scaler on {len(self.feature_columns)} features...")
        
        # Scaler 학습
        scaled_values = self.scaler.fit_transform(train_df[self.feature_columns])
        
        # DataFrame 생성
        scaled_df = pd.DataFrame(
            scaled_values,
            columns=self.feature_columns,
            index=train_df.index
        )
        
        # 제외된 컬럼 추가
        for col in exclude_columns:
            if col in train_df.columns:
                scaled_df[col] = train_df[col].values
        
        print(f"✅ Scaler fitted and transformed")
        
        return scaled_df
    
    def transform(self, df: pd.DataFrame) -> pd.DataFrame:
        """
        학습된 Scaler로 데이터 변환
        
        Args:
            df: DataFrame
        
        Returns:
            정규화된 DataFrame
        """
        if self.scaler is None or self.feature_columns is None:
            raise ValueError("Scaler not fitted. Call fit_transform() first.")
        
        # 변환
        scaled_values = self.scaler.transform(df[self.feature_columns])
        
        # DataFrame 생성
        scaled_df = pd.DataFrame(
            scaled_values,
            columns=self.feature_columns,
            index=df.index
        )
        
        # timestamp 같은 제외 컬럼 추가
        exclude_cols = [col for col in df.columns if col not in self.feature_columns]
        for col in exclude_cols:
            scaled_df[col] = df[col].values
        
        return scaled_df
    
    def save_scaler(self, filepath: str):
        """
        Scaler 파라미터 저장
        
        Args:
            filepath: 저장 경로
        """
        os.makedirs(os.path.dirname(filepath), exist_ok=True)
        
        scaler_params = {
            'scaler_type': self.scaler_type,
            'feature_columns': self.feature_columns,
        }
        
        # Scaler별 파라미터 저장
        if self.scaler_type == 'standard':
            scaler_params['mean'] = self.scaler.mean_.tolist()
            scaler_params['scale'] = self.scaler.scale_.tolist()
        elif self.scaler_type == 'minmax':
            scaler_params['min'] = self.scaler.min_.tolist()
            scaler_params['scale'] = self.scaler.scale_.tolist()
        elif self.scaler_type == 'robust':
            scaler_params['center'] = self.scaler.center_.tolist()
            scaler_params['scale'] = self.scaler.scale_.tolist()
        
        with open(filepath, 'w') as f:
            json.dump(scaler_params, f, indent=4)
        
        print(f"✅ Scaler saved to {filepath}")
    
    def load_scaler(self, filepath: str):
        """
        Scaler 파라미터 로드
        
        Args:
            filepath: 로드 경로
        """
        with open(filepath, 'r') as f:
            scaler_params = json.load(f)
        
        self.scaler_type = scaler_params['scaler_type']
        self.feature_columns = scaler_params['feature_columns']
        
        # Scaler 재생성
        if self.scaler_type == 'standard':
            self.scaler = StandardScaler()
            self.scaler.mean_ = np.array(scaler_params['mean'])
            self.scaler.scale_ = np.array(scaler_params['scale'])
        elif self.scaler_type == 'minmax':
            self.scaler = MinMaxScaler()
            self.scaler.min_ = np.array(scaler_params['min'])
            self.scaler.scale_ = np.array(scaler_params['scale'])
        elif self.scaler_type == 'robust':
            self.scaler = RobustScaler()
            self.scaler.center_ = np.array(scaler_params['center'])
            self.scaler.scale_ = np.array(scaler_params['scale'])
        
        print(f"✅ Scaler loaded from {filepath}")


class DataSaver:
    """
    데이터 저장 유틸리티
    """
    
    @staticmethod
    def save_to_csv(
        train_df: pd.DataFrame,
        val_df: pd.DataFrame,
        test_df: pd.DataFrame,
        output_dir: str = 'data/processed'
    ):
        """
        Train/Val/Test 데이터를 CSV로 저장
        
        Args:
            train_df: 학습 데이터
            val_df: 검증 데이터
            test_df: 테스트 데이터
            output_dir: 저장 디렉토리
        """
        os.makedirs(output_dir, exist_ok=True)
        
        # 저장
        train_path = os.path.join(output_dir, 'train_scaled.csv')
        val_path = os.path.join(output_dir, 'val_scaled.csv')
        test_path = os.path.join(output_dir, 'test_scaled.csv')
        
        train_df.to_csv(train_path, index=False)
        val_df.to_csv(val_path, index=False)
        test_df.to_csv(test_path, index=False)
        
        print(f"\n💾 Data saved:")
        print(f"   {train_path}")
        print(f"   {val_path}")
        print(f"   {test_path}")
    
    @staticmethod
    def save_raw_data(
        df: pd.DataFrame,
        filename: str = 'raw_data.csv',
        output_dir: str = 'data/raw'
    ):
        """
        원본 데이터 저장
        
        Args:
            df: DataFrame
            filename: 파일명
            output_dir: 저장 디렉토리
        """
        os.makedirs(output_dir, exist_ok=True)
        filepath = os.path.join(output_dir, filename)
        df.to_csv(filepath, index=False)
        print(f"💾 Raw data saved to {filepath}")
    
    @staticmethod
    def save_metadata(
        metadata: dict,
        output_dir: str = 'data/processed'
    ):
        """
        데이터 메타정보 저장
        
        Args:
            metadata: 메타데이터 딕셔너리
            output_dir: 저장 디렉토리
        """
        os.makedirs(output_dir, exist_ok=True)
        filepath = os.path.join(output_dir, 'metadata.json')
        
        with open(filepath, 'w') as f:
            json.dump(metadata, f, indent=4)
        
        print(f"📋 Metadata saved to {filepath}")


def test_preprocessor():
    """전처리 테스트"""
    print("\n" + "="*60)
    print("🧪 Testing Data Preprocessor")
    print("="*60)
    
    # 샘플 데이터 생성
    dates = pd.date_range(start='2024-01-01', periods=1000, freq='1H')
    np.random.seed(42)
    
    df = pd.DataFrame({
        'timestamp': dates,
        'feature1': np.random.randn(1000) * 100 + 50000,
        'feature2': np.random.randn(1000) * 10 + 100,
        'feature3': np.random.randn(1000) * 5 + 50,
    })
    
    # 전처리
    preprocessor = DataPreprocessor(scaler_type='standard')
    
    # 분할
    train_df, val_df, test_df = preprocessor.split_data(df)
    
    # 정규화
    train_scaled = preprocessor.fit_transform(train_df)
    val_scaled = preprocessor.transform(val_df)
    test_scaled = preprocessor.transform(test_df)
    
    print("\n📊 Scaled data sample:")
    print(train_scaled.head())
    
    # 저장 테스트
    DataSaver.save_to_csv(train_scaled, val_scaled, test_scaled, output_dir='test_output')
    
    print("\n✅ Test complete!")


if __name__ == '__main__':
    test_preprocessor()