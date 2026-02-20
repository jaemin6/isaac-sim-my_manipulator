"""
train_rl.py - 강화학습 트레이딩 V3 (좋은 거래 학습)

핵심 개선사항:
1. 최소 홀딩 시간 강제
2. 홀딩 시간 보너스
3. 수익/손실 차별화
"""

import numpy as np
import pandas as pd
import torch
import os
import argparse
from pathlib import Path
import time
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from rl.config import get_config, RLConfig
from rl.environment import TradingEnvironment
from rl.ppo_agent import PPOAgent


def load_processed_data(data_dir: str = "data/processed"):
    """전처리된 데이터 로드"""
    print("\n[Loading processed data...]")
    
    data_path = Path(data_dir)
    
    train_df = pd.read_csv(data_path / "train_scaled.csv")
    val_df = pd.read_csv(data_path / "val_scaled.csv")
    test_df = pd.read_csv(data_path / "test_scaled.csv")
    
    if 'timestamp' in train_df.columns:
        train_data = train_df.drop('timestamp', axis=1).values
        val_data = val_df.drop('timestamp', axis=1).values
        test_data = test_df.drop('timestamp', axis=1).values
    else:
        train_data = train_df.values
        val_data = val_df.values
        test_data = test_df.values
    
    print(f"\nData loaded successfully:")
    print(f"    Train: {train_data.shape}")
    print(f"    Val:   {val_data.shape}")
    print(f"    Test:  {test_data.shape}")
    
    return train_data, val_data, test_data


def comprehensive_diagnostic(train_data, config):
    """
    종합 진단 시스템 V3
    """
    print("\n" + "="*70)
    print("🔬 COMPREHENSIVE DIAGNOSTIC V3")
    print("="*70)
    
    # [1] 데이터 검증
    print(f"\n[1] Data Validation:")
    print(f"    Shape: {train_data.shape}")
    print(f"    Mean: {train_data.mean():.6f}")
    print(f"    Std: {train_data.std():.6f}")
    print(f"    Range: [{train_data.min():.4f}, {train_data.max():.4f}]")
    print(f"    Price column: [{train_data[:, 0].min():.2f}, {train_data[:, 0].max():.2f}]")
    
    # [2] 환경 테스트 (V3 - CurriculumTradingEnv 제거)
    print(f"\n[2] Environment V3 Test:")
    
    env = TradingEnvironment(
        data=train_data,
        initial_balance=config.env.initial_balance,
        transaction_fee=config.env.transaction_fee,
        window_size=config.env.window_size,
        max_position=config.env.max_position,
        min_hold_steps=config.env.min_hold_steps,  # 🆕 V3
        position_change_threshold=config.env.position_change_threshold,  # 🆕 V3
        verbose=True
    )
    print("    Using TradingEnvironment V3")
    
    obs = env.reset()
    print(f"    Observation shape: {obs.shape}")
    print(f"    Min hold steps: {config.env.min_hold_steps}")
    print(f"    Position threshold: {config.env.position_change_threshold}")
    
    # [3] 보상 함수 테스트
    print(f"\n[3] Reward Function Test (V3):")
    test_actions = {
        'Strong BUY': np.array([1.0]),
        'Mild BUY': np.array([0.3]),
        'HOLD': np.array([0.0]),
        'Mild SELL': np.array([-0.3]),
        'Strong SELL': np.array([-1.0])
    }
    
    for action_name, action in test_actions.items():
        env_test = TradingEnvironment(
            data=train_data,
            initial_balance=config.env.initial_balance,
            transaction_fee=config.env.transaction_fee,
            window_size=config.env.window_size,
            max_position=config.env.max_position,
            min_hold_steps=config.env.min_hold_steps,
            position_change_threshold=config.env.position_change_threshold
        )
        obs = env_test.reset()
        _, reward, _, info = env_test.step(action)
        print(f"    {action_name:15s} → Reward: {reward:8.4f}, Position: {info['position']:.2f}")
    
    # [4] 네트워크 테스트
    print(f"\n[4] Network Test (V3):")
    state_dim = obs.shape[0] * obs.shape[1]
    action_dim = env.action_space.shape[0]
    
    agent = PPOAgent(
        state_dim=state_dim,
        action_dim=action_dim,
        hidden_dims=config.network.hidden_dims,
        entropy_coef=config.ppo.entropy_coef,
        device=config.training.device
    )
    
    # 초기 행동 분포
    actions = []
    for i in range(100):
        action, _, _ = agent.select_action(obs, deterministic=False)
        actions.append(action[0])
    
    actions = np.array(actions)
    print(f"    Initial action distribution (100 samples):")
    print(f"      Mean: {actions.mean():.4f}")
    print(f"      Std: {actions.std():.4f}")
    print(f"      Min: {actions.min():.4f}, Max: {actions.max():.4f}")
    print(f"      Epsilon: {agent.epsilon:.4f}")
    print(f"      Noise Std: {agent.action_noise_std:.4f}")
    
    # 행동 분포 히스토그램
    bins = [-1.0, -0.5, -0.1, 0.1, 0.5, 1.0]
    hist, _ = np.histogram(actions, bins=bins + [1.1])
    print(f"    Action distribution:")
    print(f"      Strong SELL [-1.0, -0.5): {hist[0]} ({hist[0]/100*100:.0f}%)")
    print(f"      Mild SELL [-0.5, -0.1): {hist[1]} ({hist[1]/100*100:.0f}%)")
    print(f"      HOLD [-0.1, 0.1): {hist[2]} ({hist[2]/100*100:.0f}%)")
    print(f"      Mild BUY [0.1, 0.5): {hist[3]} ({hist[3]/100*100:.0f}%)")
    print(f"      Strong BUY [0.5, 1.0]: {hist[4]} ({hist[4]/100*100:.0f}%)")
    
    if actions.std() < 0.1:
        print("    ⚠️  WARNING: Action diversity too low!")
    else:
        print("    ✅ Good action diversity!")
    
    print("="*70)
    print("✅ Diagnostic complete!\n")
    
    return env, agent


def monitor_action_distribution(agent, env, n_samples=200):
    """행동 분포 모니터링"""
    obs = env.reset()
    actions = []
    
    for _ in range(n_samples):
        action, _, _ = agent.select_action(obs, deterministic=False)
        actions.append(action[0])
        obs, _, done, _ = env.step(action)
        if done:
            obs = env.reset()
    
    actions = np.array(actions)
    
    return {
        'mean': actions.mean(),
        'std': actions.std(),
        'min': actions.min(),
        'max': actions.max(),
        'epsilon': agent.epsilon,
        'noise_std': agent.action_noise_std
    }


def train(config_name: str = 'default', resume_from: str = None):
    """
    V3 학습 함수
    """
    print("\n" + "="*70)
    print("🚀 RL Trading V3 - Good Trade Learning")
    print("="*70)
    
    # 설정
    config = get_config(config_name)
    config.print_config()
    
    # 데이터
    try:
        train_data, val_data, test_data = load_processed_data(config.env.data_dir)
    except FileNotFoundError:
        print(f"\n❌ Error: Data not found at {config.env.data_dir}")
        return
    
    # 진단
    if config.training.run_diagnostic:
        env, agent = comprehensive_diagnostic(train_data, config)
    else:
        # 환경 및 에이전트 생성
        env = TradingEnvironment(
            data=train_data,
            initial_balance=config.env.initial_balance,
            transaction_fee=config.env.transaction_fee,
            window_size=config.env.window_size,
            max_position=config.env.max_position,
            min_hold_steps=config.env.min_hold_steps,
            position_change_threshold=config.env.position_change_threshold
        )
        
        obs = env.reset()
        state_dim = obs.shape[0] * obs.shape[1]
        action_dim = env.action_space.shape[0]
        
        agent = PPOAgent(
            state_dim=state_dim,
            action_dim=action_dim,
            hidden_dims=config.network.hidden_dims,
            lr=config.ppo.learning_rate,
            gamma=config.ppo.gamma,
            gae_lambda=config.ppo.gae_lambda,
            clip_epsilon=config.ppo.clip_epsilon,
            value_loss_coef=config.ppo.value_loss_coef,
            entropy_coef=config.ppo.entropy_coef,
            device=config.training.device
        )
    
    # 체크포인트 디렉토리
    os.makedirs(config.training.checkpoint_dir, exist_ok=True)
    
    # 체크포인트 로드
    if resume_from and os.path.exists(resume_from):
        print(f"\n[Resuming from {resume_from}]")
        agent.load(resume_from)
    
    # 학습 통계
    episode_rewards = []
    episode_profits = []
    episode_trades = []
    episode_lengths = []
    best_profit = -float('inf')
    no_improvement_count = 0
    
    print("\n" + "="*70)
    print("🎯 Training Loop")
    print("="*70)
    
    start_time = time.time()
    
    try:
        for episode in range(1, config.training.max_episodes + 1):
            # 에피소드 실행
            obs = env.reset()
            done = False
            episode_reward = 0
            step_count = 0
            
            states, actions, log_probs, rewards, values, dones = [], [], [], [], [], []
            
            while not done and step_count < config.training.max_steps_per_episode:
                action, log_prob, value = agent.select_action(obs, deterministic=False)
                
                next_obs, reward, done, info = env.step(action)
                
                states.append(obs.flatten())
                actions.append(action)
                log_probs.append(log_prob)
                rewards.append(reward)
                values.append(value)
                dones.append(done)
                
                obs = next_obs
                episode_reward += reward
                step_count += 1
            
            # 마지막 value
            if done:
                next_value = 0.0
            else:
                _, _, next_value = agent.select_action(obs, deterministic=False)
            
            # GAE 계산
            returns, advantages = agent.compute_gae(rewards, values, dones, next_value)
            
            # PPO 업데이트
            update_stats = agent.update(
                states=np.array(states),
                actions=np.array(actions),
                old_log_probs=np.array(log_probs),
                returns=returns,
                advantages=advantages,
                epochs=config.ppo.n_epochs,
                batch_size=config.ppo.batch_size
            )
            
            # 탐험 파라미터 감소
            agent.decay_exploration()
            
            # 통계 기록
            episode_rewards.append(episode_reward)
            episode_profits.append(info['total_profit'])
            episode_trades.append(info['num_trades'])
            episode_lengths.append(step_count)
            
            # 최고 모델 저장
            if info['total_profit'] > best_profit:
                best_profit = info['total_profit']
                agent.save(config.training.best_model_path)
                no_improvement_count = 0
            else:
                no_improvement_count += 1
            
            # 로깅
            if episode % config.training.log_interval == 0:
                avg_reward = np.mean(episode_rewards[-config.training.log_interval:])
                avg_profit = np.mean(episode_profits[-config.training.log_interval:])
                avg_trades = np.mean(episode_trades[-config.training.log_interval:])
                
                print(f"\n📈 Episode {episode}/{config.training.max_episodes}")
                print(f"   Reward: {episode_reward:.2f} (Avg: {avg_reward:.2f})")
                print(f"   Profit: ${info['total_profit']:.2f} (Avg: ${avg_profit:.2f})")
                print(f"   Trades: {info['num_trades']} (Avg: {avg_trades:.1f})")  # 🔥 거래 횟수 주목!
                print(f"   Length: {step_count}")
                print(f"   Win Rate: {info['win_rate']*100:.1f}%")
                print(f"   Epsilon: {agent.epsilon:.4f}, Noise: {agent.action_noise_std:.4f}")
                print(f"   Loss: {update_stats['total_loss']:.4f}, Entropy: {update_stats['entropy']:.4f}")
            
            # 행동 분포 체크
            if episode % config.training.check_actions_every == 0:
                action_stats = monitor_action_distribution(agent, env)
                print(f"\n🎲 Action Distribution (Episode {episode}):")
                print(f"   Mean: {action_stats['mean']:.4f}, Std: {action_stats['std']:.4f}")
                print(f"   Range: [{action_stats['min']:.4f}, {action_stats['max']:.4f}]")
            
            # 체크포인트 저장
            if episode % config.training.save_interval == 0:
                checkpoint_path = os.path.join(
                    config.training.checkpoint_dir,
                    f"checkpoint_episode_{episode}.pt"
                )
                agent.save(checkpoint_path)
            
            # 조기 종료
            if config.training.early_stopping:
                if no_improvement_count >= config.training.patience:
                    print(f"\n⚠️  Early stopping: No improvement for {config.training.patience} episodes")
                    break
        
        # 학습 완료
        elapsed_time = (time.time() - start_time) / 60
        print("\n" + "="*70)
        print("✅ Training Complete!")
        print(f"   Total time: {elapsed_time:.2f} minutes")
        print(f"   Episodes: {episode}")
        print(f"   Best profit: ${best_profit:.2f}")
        print("="*70)
        
        # 학습 그래프
        plot_training_results(episode_rewards, episode_profits, episode_trades, config)
        
        # 최종 테스트
        print("\n[Running final test...]")
        test_env = TradingEnvironment(
            data=test_data,
            initial_balance=config.env.initial_balance,
            transaction_fee=config.env.transaction_fee,
            window_size=config.env.window_size,
            max_position=config.env.max_position,
            min_hold_steps=config.env.min_hold_steps,
            position_change_threshold=config.env.position_change_threshold
        )
        
        evaluate(agent, test_env, n_episodes=config.evaluation.n_eval_episodes)
    
    except KeyboardInterrupt:
        print("\n\n⚠️  Training interrupted!")
        agent.save(os.path.join(config.training.checkpoint_dir, 'interrupted_model.pt'))


def plot_training_results(rewards, profits, trades, config):
    """학습 결과 시각화"""
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    
    # Rewards
    axes[0].plot(rewards, alpha=0.3, label='Episode Reward')
    axes[0].plot(pd.Series(rewards).rolling(50).mean(), label='Moving Avg (50)')
    axes[0].set_title('Training Rewards')
    axes[0].set_xlabel('Episode')
    axes[0].set_ylabel('Reward')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)
    
    # Profits
    axes[1].plot(profits, alpha=0.3, label='Episode Profit')
    axes[1].plot(pd.Series(profits).rolling(50).mean(), label='Moving Avg (50)')
    axes[1].axhline(y=0, color='r', linestyle='--', alpha=0.5)
    axes[1].set_title('Training Profits')
    axes[1].set_xlabel('Episode')
    axes[1].set_ylabel('Profit ($)')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)
    
    # Trades
    axes[2].plot(trades, alpha=0.3, label='Episode Trades')
    axes[2].plot(pd.Series(trades).rolling(50).mean(), label='Moving Avg (50)')
    axes[2].set_title('Number of Trades')
    axes[2].set_xlabel('Episode')
    axes[2].set_ylabel('Trades')
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    plot_path = os.path.join(config.training.checkpoint_dir, 'training_results_v3.png')
    plt.savefig(plot_path, dpi=150)
    print(f"📊 Training plot saved to {plot_path}")
    plt.close()


def evaluate(agent, env, n_episodes=10):
    """모델 평가"""
    print("\n" + "="*70)
    print("🧪 Evaluation")
    print("="*70)
    
    test_rewards = []
    test_profits = []
    test_trades = []
    test_win_rates = []
    
    for ep in range(1, n_episodes + 1):
        obs = env.reset()
        done = False
        episode_reward = 0
        
        while not done:
            action, _, _ = agent.select_action(obs, deterministic=True)
            obs, reward, done, info = env.step(action)
            episode_reward += reward
        
        test_rewards.append(episode_reward)
        test_profits.append(info['total_profit'])
        test_trades.append(info['num_trades'])
        test_win_rates.append(info['win_rate'])
        
        print(f"Episode {ep}/{n_episodes}: "
              f"Reward={episode_reward:.2f}, "
              f"Profit=${info['total_profit']:.2f}, "
              f"Trades={info['num_trades']}, "
              f"WinRate={info['win_rate']*100:.1f}%")
    
    print("\n" + "="*70)
    print("📊 Evaluation Results:")
    print(f"   Avg Reward: {np.mean(test_rewards):.2f} ± {np.std(test_rewards):.2f}")
    print(f"   Avg Profit: ${np.mean(test_profits):.2f} ± ${np.std(test_profits):.2f}")
    print(f"   Avg Trades: {np.mean(test_trades):.1f}")
    print(f"   Avg Win Rate: {np.mean(test_win_rates)*100:.1f}%")
    print(f"   Win Episodes: {sum(p > 0 for p in test_profits)}/{n_episodes}")
    print("="*70)


def main():
    """메인 함수"""
    parser = argparse.ArgumentParser(description='RL Trading V3')
    
    parser.add_argument('--config', type=str, default='default',
                        choices=['default', 'conservative', 'aggressive', 'balanced'],
                        help='Config preset')
    
    parser.add_argument('--resume', type=str, default=None,
                        help='Checkpoint to resume from')
    
    args = parser.parse_args()
    
    # GPU
    device = 'CUDA' if torch.cuda.is_available() else 'CPU'
    print(f"\n[Device: {device}]")
    if torch.cuda.is_available():
        print(f"    GPU: {torch.cuda.get_device_name(0)}")
    
    # 학습 실행
    train(config_name=args.config, resume_from=args.resume)


if __name__ == '__main__':
    main()