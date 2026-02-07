"""
Phase 4: 강화학습 실행 스크립트 구조
"""
import argparse
from rl.trainer import create_trainer_from_data
from rl.config import get_config

def load_processed_data(data_dir):
    """CSV 데이터를 불러와 numpy 배열로 변환"""
    # ... 데이터 로드 및 분할 로직
    return train_data, val_data, test_data

def train(config_name, resume_from=None):
    """설정 로드 -> 트레이너 생성 -> 학습 시작 -> 결과 시각화"""
    # 1. config = get_config(config_name)
    # 2. trainer = create_trainer_from_data(...)
    # 3. trainer.train()
    # 4. trainer.plot_training_progress()
    pass

def evaluate(model_path, config_name, n_episodes):
    """학습된 모델 로드 -> 테스트 환경 실행 -> 성과 지표 출력"""
    # 1. agent.load(model_path)
    # 2. 에피소드 반복 돌며 수익률/승률 계산
    pass

def main():
    """명령줄 인자(CLI)를 분석하여 train 또는 eval 모드 실행"""
    # --mode, --config, --resume 등의 인자 설정
    pass

if __name__ == '__main__':
    main()