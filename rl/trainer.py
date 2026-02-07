import numpy as np
import torch
from typing import Dict, List, Tuple, Optional

class RLTrainer:
    """환경, 에이전트, 버퍼를 통합하여 학습 루프(Train/Eval/Test)를 실행"""
    
    def __init__(self, env, agent, config, val_env=None):
        # 1. 환경 및 에이전트 초기화
        # 2. 결과 기록용 리스트 및 체크포인트 디렉토리 생성
        pass

    def train(self) -> Dict:
        """메인 학습 루프 (Episode -> Run -> Train Step -> Log -> Eval)"""
        # 에피소드 반복문 실행
        # _run_episode() 호출 후 데이터 수집
        # 버퍼가 차면 _train_step() 호출하여 모델 업데이트
        # 주기적으로 _evaluate() 및 모델 저장
        pass

    def _run_episode(self) -> Tuple[float, int, float]:
        """실제 환경에서 1개 에피소드 진행 및 버퍼에 데이터 저장"""
        pass

    def _train_step(self) -> Dict[str, float]:
        """버퍼의 데이터를 에이전트에게 전달하여 PPO 알고리즘 업데이트"""
        pass

    def _evaluate(self, n_episodes=None) -> Tuple[float, float]:
        """검증 데이터셋에서 모델 성능(수익률 등) 평가"""
        pass

    def _save_training_stats(self):
        """학습 결과(보상, 수익 내역)를 JSON으로 저장"""
        pass

    def plot_training_progress(self, save_path=None):
        """보상 및 수익 곡선 시각화 그래프 생성"""
        pass

    def test(self, test_env, n_episodes=10) -> Dict:
        """최종 테스트 환경에서 모델 성능 검증 및 리포트 생성"""
        pass

def create_trainer_from_data(train_data, val_data=None, config=None) -> RLTrainer:
    """데이터와 설정을 받아 트레이너 객체를 생성하는 Factory 함수"""
    pass