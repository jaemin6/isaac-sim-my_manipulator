"""
train_rl.py - Phase 4: 강화학습 트레이딩

암호화폐 트레이딩을 위한 PPO 강화학습 학습 스크립트
"""

import numpy as np
import pandas as pd
import torch
import os
import argparse
from pathlib import Path

from rl.trainer import RLTrainer, create_trainer_from_data
from rl.config import RLConfig, get_config
from rl.environment import TradingEnvironment
from rl.ppo_agent import PPOAgent


def load_processed_data(data_dir: str = "data/processed"):
    """
    전처리된 데이터 로드
    
    Args:
        data_dir: 데이터 디렉토리 경로
    
    Returns:
        train_data, val_data, test_data
    """
    print("\n[Loading processed data...]")
    
    data_path = Path(data_dir)
    
    # CSV 파일 로드
    train_df = pd.read_csv(data_path / "train_scaled.csv")
    val_df = pd.read_csv(data_path / "val_scaled.csv")
    test_df = pd.read_csv(data_path / "test_scaled.csv")
    
    # numpy 배열로 변환 (timestamp 제외)
    if 'timestamp' in train_df.columns:
        train_data = train_df.drop('timestamp', axis=1).values
        val_data = val_df.drop('timestamp', axis=1).values
        test_data = test_df.drop('timestamp', axis=1).values
    else:
        train_data = train_df.values
        val_data = val_df.values
        test_data = test_df.values
    
    print(f"Data loaded successfully:")
    print(f"    Train: {train_data.shape}")
    print(f"    Val:   {val_data.shape}")
    print(f"    Test:  {test_data.shape}")
    
    return train_data, val_data, test_data


def train(config_name: str = 'default', resume_from: str = None):
    """
    강화학습 학습 실행
    
    Args:
        config_name: 설정 이름 ('default', 'conservative', 'aggressive', 'recurrent')
        resume_from: 체크포인트 경로 (이어서 학습)
    """
    print("\n" + "="*60)
    print("Phase 4: Reinforcement Learning Training Start")
    print("="*60)
    
    # 설정 로드
    config = get_config(config_name)
    config.print_config()
    
    # 데이터 로드
    try:
        train_data, val_data, test_data = load_processed_data(config.env.data_dir)
    except FileNotFoundError as e:
        print(f"\nError: Processed data not found.")
        print(f"    Please run Phase 1-3 first to prepare data.")
        print(f"    Path: {config.env.data_dir}")
        return
    
    # 트레이너 생성
    print("\n[Creating trainer...]")
    trainer = create_trainer_from_data(
        train_data=train_data,
        val_data=val_data,
        config=config
    )
    
    # 체크포인트에서 이어서 학습
    if resume_from and os.path.exists(resume_from):
        print(f"\n[Resuming from checkpoint: {resume_from}]")
        trainer.agent.load(resume_from)
    
    # 학습 시작
    print("\n" + "="*60)
    print("Training process initiated...")
    print("="*60)
    
    try:
        stats = trainer.train()
        
        # 학습 진행 상황 시각화
        print("\n[Plotting training progress...]")
        plot_path = os.path.join(config.training.checkpoint_dir, 'training_progress.png')
        trainer.plot_training_progress(save_path=plot_path)
        
        # 테스트 환경 생성
        print("\n[Creating test environment...]")
        test_env = TradingEnvironment(
            data=test_data,
            initial_balance=config.env.initial_balance,
            transaction_fee=config.env.transaction_fee,
            window_size=config.env.window_size,
            max_position=config.env.max_position
        )
        
        # 테스트 실행
        print("\n[Running final test...]")
        test_results = trainer.test(test_env, n_episodes=config.evaluation.n_eval_episodes)
        
        print("\n" + "="*60)
        print("Training complete!")
        print(f"    Best model: {config.training.best_model_path}")
        print(f"    Checkpoints: {config.training.checkpoint_dir}")
        print("="*60)
        
    except KeyboardInterrupt:
        print("\n\nWarning: Training interrupted by user")
        print("    Saving current model...")
        interrupt_path = os.path.join(config.training.checkpoint_dir, 'interrupted_model.pt')
        trainer.agent.save(interrupt_path)
        print(f"    Model saved to {interrupt_path}")
    
    except Exception as e:
        print(f"\nError during training: {e}")
        import traceback
        traceback.print_exc()


def evaluate(model_path: str, config_name: str = 'default', n_episodes: int = 10):
    """
    학습된 모델 평가
    
    Args:
        model_path: 모델 체크포인트 경로
        config_name: 설정 이름
        n_episodes: 평가 에피소드 수
    """
    print("\n" + "="*60)
    print("Model Evaluation Start")
    print("="*60)
    
    # 설정 로드
    config = get_config(config_name)
    
    # 데이터 로드
    try:
        _, _, test_data = load_processed_data(config.env.data_dir)
    except FileNotFoundError:
        print(f"Error: Data not found.")
        return
    
    # 테스트 환경 생성
    test_env = TradingEnvironment(
        data=test_data,
        initial_balance=config.env.initial_balance,
        transaction_fee=config.env.transaction_fee,
        window_size=config.env.window_size,
        max_position=config.env.max_position
    )
    
    # 상태 및 행동 차원
    state_dim = test_env.observation_space.shape[0] * test_env.observation_space.shape[1]
    action_dim = test_env.action_space.shape[0]
    
    # 에이전트 생성
    agent = PPOAgent(
        state_dim=state_dim,
        action_dim=action_dim,
        hidden_dims=config.network.hidden_dims,
        device=config.training.device
    )
    
    # 모델 로드
    if not os.path.exists(model_path):
        print(f"Error: Model file not found: {model_path}")
        return
    
    print(f"[Loading model from {model_path}]")
    agent.load(model_path)
    
    # 평가 실행
    print(f"\n[Running evaluation ({n_episodes} episodes)...]")
    
    test_rewards = []
    test_profits = []
    test_balances = []
    
    for ep in range(n_episodes):
        state = test_env.reset()
        done = False
        episode_reward = 0
        
        while not done:
            action, _, _ = agent.select_action(state, deterministic=True)
            next_state, reward, done, info = test_env.step(action)
            episode_reward += reward
            state = next_state
        
        test_rewards.append(episode_reward)
        test_profits.append(info.get('total_profit', 0))
        test_balances.append(info.get('balance', 0))
        
        print(f"Episode {ep+1}/{n_episodes}: "
              f"Reward={episode_reward:.2f}, "
              f"Profit=${info.get('total_profit', 0):.2f}, "
              f"Balance=${info.get('balance', 0):.2f}")
    
    # 결과 통계
    print("\n" + "="*60)
    print("Evaluation Results Summary:")
    print(f"    Avg Reward: {np.mean(test_rewards):.2f} +/- {np.std(test_rewards):.2f}")
    print(f"    Avg Profit: ${np.mean(test_profits):.2f} +/- ${np.std(test_profits):.2f}")
    print(f"    Avg Balance: ${np.mean(test_balances):.2f}")
    print(f"    Win Rate: {np.mean([p > 0 for p in test_profits])*100:.2f}%")
    print(f"    Max Profit: ${max(test_profits):.2f}")
    print(f"    Min Profit: ${min(test_profits):.2f}")
    print("="*60)


def main():
    """메인 함수"""
    parser = argparse.ArgumentParser(description='Phase 4: RL Trading System')
    
    parser.add_argument('--mode', type=str, default='train',
                        choices=['train', 'eval'],
                        help='Execution mode (train/eval)')
    
    parser.add_argument('--config', type=str, default='default',
                        choices=['default', 'conservative', 'aggressive', 'recurrent'],
                        help='Config preset')
    
    parser.add_argument('--resume', type=str, default=None,
                        help='Checkpoint path to resume training')
    
    parser.add_argument('--model', type=str, default='checkpoints/rl/best_model.pt',
                        help='Model path for evaluation')
    
    parser.add_argument('--episodes', type=int, default=10,
                        help='Number of evaluation episodes')
    
    args = parser.parse_args()
    
    # GPU 사용 가능 여부 확인
    device_type = 'CUDA' if torch.cuda.is_available() else 'CPU'
    print(f"\n[Device Info: {device_type}]")
    if torch.cuda.is_available():
        print(f"    GPU Model: {torch.cuda.get_device_name(0)}")
    
    # 모드에 따라 실행
    if args.mode == 'train':
        train(config_name=args.config, resume_from=args.resume)
    elif args.mode == 'eval':
        evaluate(model_path=args.model, config_name=args.config, n_episodes=args.episodes)


if __name__ == '__main__':
    main()