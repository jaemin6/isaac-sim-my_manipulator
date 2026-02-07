"""
test_rl.py - Phase 4: 학습된 모델 테스트

학습된 강화학습 모델을 테스트하고 백테스팅 결과를 시각화합니다.
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import torch
import os
import argparse
from pathlib import Path

from rl.environment import TradingEnvironment
from rl.ppo_agent import PPOAgent
from rl.config import get_config


def backtest_model(
    model_path: str,
    data_path: str = "data/processed/test_scaled.csv",
    config_name: str = 'default',
    visualize: bool = True
):
    """
    학습된 모델로 백테스팅 실행
    
    Args:
        model_path: 모델 체크포인트 경로
        data_path: 테스트 데이터 경로
        config_name: 설정 이름
        visualize: 결과 시각화 여부
    """
    print("\n" + "="*60)
    print("Backtesting Start")
    print("="*60)
    
    # 설정 로드
    config = get_config(config_name)
    
    # 데이터 로드
    print(f"\n[Loading data from {data_path}]")
    df = pd.read_csv(data_path)
    
    if 'timestamp' in df.columns:
        timestamps = df['timestamp'].values
        data = df.drop('timestamp', axis=1).values
    else:
        timestamps = None
        data = df.values
    
    print(f"Data loaded successfully: {data.shape}")
    
    # 환경 생성
    env = TradingEnvironment(
        data=data,
        initial_balance=config.env.initial_balance,
        transaction_fee=config.env.transaction_fee,
        window_size=config.env.window_size,
        max_position=config.env.max_position
    )
    
    # 에이전트 생성 및 모델 로드
    state_dim = env.observation_space.shape[0] * env.observation_space.shape[1]
    action_dim = env.action_space.shape[0]
    
    agent = PPOAgent(
        state_dim=state_dim,
        action_dim=action_dim,
        hidden_dims=config.network.hidden_dims,
        device=config.training.device
    )
    
    if not os.path.exists(model_path):
        print(f"Error: Model file not found: {model_path}")
        return
    
    print(f"[Loading model from {model_path}]")
    agent.load(model_path)
    
    # 백테스팅 실행
    print("\n[Running backtest...]")
    state = env.reset()
    done = False
    
    # 추적할 데이터
    portfolio_values = [config.env.initial_balance]
    positions = [0]
    actions_taken = []
    prices = [env._get_current_price()]
    
    step = 0
    while not done:
        # 행동 선택 (결정적)
        action, _, _ = agent.select_action(state, deterministic=True)
        
        # 환경 스텝
        next_state, reward, done, info = env.step(action)
        
        # 데이터 기록
        portfolio_value = env.get_portfolio_value()
        portfolio_values.append(portfolio_value)
        positions.append(env.position)
        actions_taken.append(action[0])
        prices.append(env._get_current_price())
        
        state = next_state
        step += 1
        
        if step % 100 == 0:
            print(f"    Step {step}: Portfolio=${portfolio_value:.2f}, Position={env.position:.2f}")
    
    # 최종 결과
    final_profit = info.get('total_profit', 0)
    final_balance = info.get('balance', 0)
    num_trades = len(env.trade_history)
    
    print("\n" + "="*60)
    print("Backtest Results Summary:")
    print(f"    Initial Balance: ${config.env.initial_balance:.2f}")
    print(f"    Final Balance:   ${final_balance:.2f}")
    print(f"    Total Profit:    ${final_profit:.2f}")
    print(f"    Return:          {(final_profit/config.env.initial_balance)*100:.2f}%")
    print(f"    Total Trades:    {num_trades}")
    print(f"    Total Steps:     {step}")
    
    # 성과 지표 계산
    returns = np.diff(portfolio_values) / portfolio_values[:-1]
    sharpe_ratio = np.mean(returns) / (np.std(returns) + 1e-8) * np.sqrt(252)  # 연율화
    max_drawdown = calculate_max_drawdown(portfolio_values)
    
    print(f"    Sharpe Ratio:    {sharpe_ratio:.3f}")
    print(f"    Max Drawdown:    {max_drawdown*100:.2f}%")
    print("="*60)
    
    # 시각화
    if visualize:
        print("\n[Plotting backtest results...]")
        plot_backtest_results(
            portfolio_values=portfolio_values,
            positions=positions,
            prices=prices,
            trades=env.trade_history,
            timestamps=timestamps,
            save_path='backtest_results.png'
        )
    
    return {
        'final_profit': final_profit,
        'final_balance': final_balance,
        'num_trades': num_trades,
        'sharpe_ratio': sharpe_ratio,
        'max_drawdown': max_drawdown,
        'portfolio_values': portfolio_values
    }


def calculate_max_drawdown(portfolio_values):
    """최대 낙폭 계산"""
    portfolio_values = np.array(portfolio_values)
    running_max = np.maximum.accumulate(portfolio_values)
    drawdown = (portfolio_values - running_max) / running_max
    return abs(np.min(drawdown))


def plot_backtest_results(
    portfolio_values,
    positions,
    prices,
    trades,
    timestamps=None,
    save_path='backtest_results.png'
):
    """백테스팅 결과 시각화"""
    fig, axes = plt.subplots(3, 1, figsize=(15, 12))
    
    steps = range(len(portfolio_values))
    
    # 1. 포트폴리오 가치
    axes[0].plot(steps, portfolio_values, label='Portfolio Value', linewidth=2)
    axes[0].axhline(y=portfolio_values[0], color='r', linestyle='--', 
                    alpha=0.5, label='Initial Balance')
    axes[0].set_ylabel('Portfolio Value ($)')
    axes[0].set_title('Portfolio Value Over Time')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)
    
    # 2. 가격 및 거래
    axes[1].plot(steps, prices, label='Price', color='blue', alpha=0.7)
    
    # 거래 표시
    for trade in trades:
        if trade['type'] == 'open':
            color = 'green' if trade['position'] > 0 else 'red'
            marker = '^' if trade['position'] > 0 else 'v'
            axes[1].scatter(trade['step'], trade['entry_price'], 
                          color=color, marker=marker, s=100, zorder=5,
                          label=f"{'Long' if trade['position'] > 0 else 'Short'} Entry")
        elif trade['type'] == 'close':
            axes[1].scatter(trade['step'], trade['exit_price'],
                          color='black', marker='x', s=100, zorder=5,
                          label='Close')
    
    axes[1].set_ylabel('Price')
    axes[1].set_title('Price and Trades')
    # 범례 중복 제거
    handles, labels = axes[1].get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    axes[1].legend(by_label.values(), by_label.keys())
    axes[1].grid(True, alpha=0.3)
    
    # 3. 포지션
    axes[2].fill_between(steps, 0, positions, 
                         where=np.array(positions) > 0,
                         color='green', alpha=0.3, label='Long Position')
    axes[2].fill_between(steps, 0, positions,
                         where=np.array(positions) < 0,
                         color='red', alpha=0.3, label='Short Position')
    axes[2].axhline(y=0, color='black', linestyle='-', linewidth=0.5)
    axes[2].set_ylabel('Position')
    axes[2].set_xlabel('Step')
    axes[2].set_title('Position Over Time')
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    print(f"Plot saved successfully to {save_path}")
    plt.close()


def compare_models(model_paths, labels, data_path="data/processed/test_scaled.csv"):
    """여러 모델 비교"""
    print("\n" + "="*60)
    print("Model Comparison")
    print("="*60)
    
    results = []
    
    for model_path, label in zip(model_paths, labels):
        print(f"\n[Testing {label}...]")
        result = backtest_model(model_path, data_path, visualize=False)
        result['label'] = label
        results.append(result)
    
    # 비교 테이블 출력
    print("\n" + "="*60)
    print("Comparison Summary:")
    print("-" * 60)
    print(f"{'Model':<20} {'Profit ($)':<15} {'Return (%)':<15} {'Sharpe':<10} {'Drawdown (%)':<15}")
    print("-" * 60)
    
    for result in results:
        profit = result['final_profit']
        return_pct = (profit / 10000) * 100  # 초기 자본 10000 가정
        sharpe = result['sharpe_ratio']
        drawdown = result['max_drawdown'] * 100
        
        print(f"{result['label']:<20} {profit:<15.2f} {return_pct:<15.2f} {sharpe:<10.3f} {drawdown:<15.2f}")
    
    print("="*60)


def main():
    parser = argparse.ArgumentParser(description='Phase 4: RL Model Testing and Backtesting')
    
    parser.add_argument('--model', type=str, default='checkpoints/rl/best_model.pt',
                        help='Model path')
    
    parser.add_argument('--data', type=str, default='data/processed/test_scaled.csv',
                        help='Test data path')
    
    parser.add_argument('--config', type=str, default='default',
                        choices=['default', 'conservative', 'aggressive', 'recurrent'],
                        help='Config preset')
    
    parser.add_argument('--no-plot', action='store_true',
                        help='Disable visualization')
    
    args = parser.parse_args()
    
    # 백테스팅 실행
    backtest_model(
        model_path=args.model,
        data_path=args.data,
        config_name=args.config,
        visualize=not args.no_plot
    )


if __name__ == '__main__':
    main()