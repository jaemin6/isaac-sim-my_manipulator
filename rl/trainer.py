# rl/trainer.py
import numpy as np
import torch
import os
import time
from typing import Dict, List, Tuple, Optional
from tqdm import tqdm
import matplotlib.pyplot as plt
import json

from .environment import TradingEnvironment, MultiAssetTradingEnvironment
from .ppo_agent import PPOAgent
from .replay_buffer import RolloutBuffer, EpisodeBuffer
from .config import RLConfig


class RLTrainer:
    """
    강화학습 트레이너
    
    환경, 에이전트, 버퍼를 통합하여 학습 루프를 실행합니다.
    """
    
    def __init__(
        self,
        env: TradingEnvironment,
        agent: PPOAgent,
        config: RLConfig,
        val_env: Optional[TradingEnvironment] = None
    ):
        """
        Args:
            env: 학습용 환경
            agent: PPO 에이전트
            config: 설정
            val_env: 검증용 환경 (optional)
        """
        self.env = env
        self.agent = agent
        self.config = config
        self.val_env = val_env
        
        # 버퍼
        self.buffer = RolloutBuffer(buffer_size=config.ppo.n_steps)
        self.episode_buffer = EpisodeBuffer()
        
        # 학습 통계
        self.episode_rewards = []
        self.episode_lengths = []
        self.episode_profits = []
        self.eval_rewards = []
        self.eval_profits = []
        
        # 최고 성능 추적
        self.best_eval_reward = -np.inf
        self.best_eval_profit = -np.inf
        self.episodes_without_improvement = 0
        
        # 체크포인트 디렉토리 생성
        os.makedirs(config.training.checkpoint_dir, exist_ok=True)
        
        # 시드 설정
        if config.training.seed is not None:
            self._set_seed(config.training.seed)
        
        print("🚀 RLTrainer initialized!")
        print(f"   Device: {agent.device}")
        print(f"   Checkpoint dir: {config.training.checkpoint_dir}")
    
    def _set_seed(self, seed: int):
        """랜덤 시드 설정"""
        np.random.seed(seed)
        torch.manual_seed(seed)
        if torch.cuda.is_available():
            torch.cuda.manual_seed(seed)
            torch.cuda.manual_seed_all(seed)
    
    def train(self) -> Dict[str, List[float]]:
        """
        학습 실행
        
        Returns:
            학습 통계 딕셔너리
        """
        print("\n" + "="*60)
        print("🎓 Training Start!")
        print("="*60)
        
        start_time = time.time()
        global_step = 0
        
        for episode in range(1, self.config.training.max_episodes + 1):
            episode_start_time = time.time()
            
            # 에피소드 실행
            episode_reward, episode_length, episode_profit = self._run_episode()
            
            # 통계 저장
            self.episode_rewards.append(episode_reward)
            self.episode_lengths.append(episode_length)
            self.episode_profits.append(episode_profit)
            self.agent.add_episode_stats(episode_reward, episode_length, episode_profit)
            
            global_step += episode_length
            
            # 버퍼가 충분히 찼으면 학습
            if self.buffer.is_full():
                train_stats = self._train_step()
            else:
                train_stats = {}
            
            # 로깅
            if episode % self.config.training.log_interval == 0:
                self._log_progress(episode, episode_reward, episode_profit, 
                                 episode_length, train_stats, episode_start_time)
            
            # 평가
            if episode % self.config.training.eval_interval == 0 and self.val_env is not None:
                eval_reward, eval_profit = self._evaluate()
                self.eval_rewards.append(eval_reward)
                self.eval_profits.append(eval_profit)
                
                print(f"\n📊 Evaluation:")
                print(f"   Avg Reward: {eval_reward:.2f}")
                print(f"   Avg Profit: ${eval_profit:.2f}")
                
                # 최고 성능 체크
                if eval_profit > self.best_eval_profit:
                    self.best_eval_profit = eval_profit
                    self.episodes_without_improvement = 0
                    
                    # 최고 모델 저장
                    best_path = self.config.training.best_model_path
                    self.agent.save(best_path)
                    print(f"   ✨ New best model saved! (Profit: ${eval_profit:.2f})")
                else:
                    self.episodes_without_improvement += self.config.training.eval_interval
            
            # 모델 저장
            if episode % self.config.training.save_interval == 0:
                checkpoint_path = os.path.join(
                    self.config.training.checkpoint_dir,
                    f"checkpoint_episode_{episode}.pt"
                )
                self.agent.save(checkpoint_path)
            
            # 조기 종료
            if self.config.training.early_stopping:
                if self.episodes_without_improvement >= self.config.training.patience:
                    print(f"\n⏹️  Early stopping triggered (no improvement for {self.config.training.patience} episodes)")
                    break
        
        total_time = time.time() - start_time
        
        print("\n" + "="*60)
        print("✅ Training Complete!")
        print(f"   Total time: {total_time/60:.2f} minutes")
        print(f"   Episodes: {episode}")
        print(f"   Best profit: ${self.best_eval_profit:.2f}")
        print("="*60)
        
        # 최종 통계 저장
        self._save_training_stats()
        
        return {
            'episode_rewards': self.episode_rewards,
            'episode_profits': self.episode_profits,
            'eval_rewards': self.eval_rewards,
            'eval_profits': self.eval_profits
        }
    
    def _run_episode(self) -> Tuple[float, int, float]:
        """
        에피소드 실행
        
        Returns:
            episode_reward: 총 보상
            episode_length: 에피소드 길이
            episode_profit: 총 수익
        """
        state = self.env.reset()
        done = False
        episode_reward = 0
        episode_length = 0
        
        while not done and episode_length < self.config.training.max_steps_per_episode:
            # 행동 선택
            action, log_prob, value = self.agent.select_action(state)
            
            # 환경 스텝
            next_state, reward, done, info = self.env.step(action)
            
            # 버퍼에 저장
            self.buffer.add(
                state=state.flatten(),
                action=action,
                reward=reward,
                value=value,
                log_prob=log_prob,
                done=done
            )
            
            episode_reward += reward
            episode_length += 1
            state = next_state
        
        episode_profit = info.get('total_profit', 0)
        
        return episode_reward, episode_length, episode_profit
    
    def _train_step(self) -> Dict[str, float]:
        """
        학습 스텝 (PPO 업데이트)
        
        Returns:
            학습 통계
        """
        # 버퍼에서 데이터 가져오기
        states, actions, rewards, values, log_probs, dones = self.buffer.get()
        
        # 마지막 상태의 가치 계산 (bootstrap)
        with torch.no_grad():
            last_state = states[-1]
            _, _, last_value = self.agent.select_action(last_state.reshape(1, -1))
        
        # GAE 계산
        returns, advantages = self.agent.compute_gae(
            rewards=rewards.tolist(),
            values=values.tolist(),
            dones=dones.tolist(),
            next_value=last_value
        )
        
        # PPO 업데이트
        train_stats = self.agent.update(
            states=states,
            actions=actions,
            old_log_probs=log_probs,
            returns=returns,
            advantages=advantages,
            epochs=self.config.ppo.n_epochs,
            batch_size=self.config.ppo.batch_size
        )
        
        # 버퍼 리셋
        self.buffer.reset()
        
        return train_stats
    
    def _evaluate(self, n_episodes: Optional[int] = None) -> Tuple[float, float]:
        """
        검증 환경에서 평가
        
        Args:
            n_episodes: 평가 에피소드 수
        
        Returns:
            avg_reward: 평균 보상
            avg_profit: 평균 수익
        """
        if n_episodes is None:
            n_episodes = self.config.evaluation.n_eval_episodes
        
        eval_rewards = []
        eval_profits = []
        
        for _ in range(n_episodes):
            state = self.val_env.reset()
            done = False
            episode_reward = 0
            
            while not done:
                # 결정적 행동 선택 (평가 시)
                action, _, _ = self.agent.select_action(
                    state,
                    deterministic=self.config.evaluation.deterministic
                )
                
                next_state, reward, done, info = self.val_env.step(action)
                episode_reward += reward
                state = next_state
            
            eval_rewards.append(episode_reward)
            eval_profits.append(info.get('total_profit', 0))
        
        avg_reward = np.mean(eval_rewards)
        avg_profit = np.mean(eval_profits)
        
        return avg_reward, avg_profit
    
    def _log_progress(
        self,
        episode: int,
        reward: float,
        profit: float,
        length: int,
        train_stats: Dict[str, float],
        episode_start_time: float
    ):
        """진행 상황 로깅"""
        episode_time = time.time() - episode_start_time
        
        # 최근 성능
        recent_rewards = self.episode_rewards[-10:]
        recent_profits = self.episode_profits[-10:]
        
        print(f"\n📈 Episode {episode}")
        print(f"   Reward: {reward:.2f} (Avg: {np.mean(recent_rewards):.2f})")
        print(f"   Profit: ${profit:.2f} (Avg: ${np.mean(recent_profits):.2f})")
        print(f"   Length: {length}")
        print(f"   Time: {episode_time:.2f}s")
        
        if train_stats:
            print(f"   Policy Loss: {train_stats.get('policy_loss', 0):.4f}")
            print(f"   Value Loss: {train_stats.get('value_loss', 0):.4f}")
            print(f"   Entropy: {train_stats.get('entropy', 0):.4f}")
            print(f"   KL Divergence: {train_stats.get('approx_kl', 0):.4f}")
    
    def _save_training_stats(self):
        """학습 통계 저장"""
        stats = {
            'episode_rewards': self.episode_rewards,
            'episode_profits': self.episode_profits,
            'episode_lengths': self.episode_lengths,
            'eval_rewards': self.eval_rewards,
            'eval_profits': self.eval_profits,
            'best_eval_profit': float(self.best_eval_profit),
            'config': self.config.to_dict()
        }
        
        stats_path = os.path.join(self.config.training.checkpoint_dir, 'training_stats.json')
        with open(stats_path, 'w') as f:
            json.dump(stats, f, indent=4)
        
        print(f"📊 Training stats saved to {stats_path}")
    
    def plot_training_progress(self, save_path: Optional[str] = None):
        """학습 진행 상황 시각화"""
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        
        # 에피소드 보상
        axes[0, 0].plot(self.episode_rewards, alpha=0.6, label='Episode Reward')
        axes[0, 0].plot(self._moving_average(self.episode_rewards, 10), 
                        label='MA(10)', linewidth=2)
        axes[0, 0].set_xlabel('Episode')
        axes[0, 0].set_ylabel('Reward')
        axes[0, 0].set_title('Episode Rewards')
        axes[0, 0].legend()
        axes[0, 0].grid(True, alpha=0.3)
        
        # 에피소드 수익
        axes[0, 1].plot(self.episode_profits, alpha=0.6, label='Episode Profit')
        axes[0, 1].plot(self._moving_average(self.episode_profits, 10),
                        label='MA(10)', linewidth=2)
        axes[0, 1].axhline(y=0, color='r', linestyle='--', alpha=0.5)
        axes[0, 1].set_xlabel('Episode')
        axes[0, 1].set_ylabel('Profit ($)')
        axes[0, 1].set_title('Episode Profits')
        axes[0, 1].legend()
        axes[0, 1].grid(True, alpha=0.3)
        
        # 평가 성능
        if len(self.eval_rewards) > 0:
            eval_episodes = np.arange(
                self.config.training.eval_interval,
                len(self.eval_rewards) * self.config.training.eval_interval + 1,
                self.config.training.eval_interval
            )
            axes[1, 0].plot(eval_episodes, self.eval_rewards, 'o-', label='Eval Reward')
            axes[1, 0].set_xlabel('Episode')
            axes[1, 0].set_ylabel('Reward')
            axes[1, 0].set_title('Evaluation Rewards')
            axes[1, 0].legend()
            axes[1, 0].grid(True, alpha=0.3)
            
            axes[1, 1].plot(eval_episodes, self.eval_profits, 'o-', label='Eval Profit')
            axes[1, 1].axhline(y=0, color='r', linestyle='--', alpha=0.5)
            axes[1, 1].set_xlabel('Episode')
            axes[1, 1].set_ylabel('Profit ($)')
            axes[1, 1].set_title('Evaluation Profits')
            axes[1, 1].legend()
            axes[1, 1].grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"📊 Training plot saved to {save_path}")
        else:
            plt.savefig(os.path.join(self.config.training.checkpoint_dir, 'training_progress.png'),
                       dpi=150, bbox_inches='tight')
        
        plt.close()
    
    @staticmethod
    def _moving_average(data: List[float], window: int) -> np.ndarray:
        """이동 평균 계산"""
        if len(data) < window:
            return np.array(data)
        return np.convolve(data, np.ones(window)/window, mode='valid')
    
    def test(self, test_env: TradingEnvironment, n_episodes: int = 10) -> Dict[str, float]:
        """
        테스트 환경에서 평가
        
        Args:
            test_env: 테스트 환경
            n_episodes: 테스트 에피소드 수
        
        Returns:
            테스트 결과 딕셔너리
        """
        print("\n" + "="*60)
        print("🧪 Testing Start!")
        print("="*60)
        
        test_rewards = []
        test_profits = []
        test_balances = []
        all_trades = []
        
        for ep in range(n_episodes):
            state = test_env.reset()
            done = False
            episode_reward = 0
            episode_trades = []
            
            while not done:
                action, _, _ = self.agent.select_action(state, deterministic=True)
                next_state, reward, done, info = test_env.step(action)
                
                episode_reward += reward
                state = next_state
            
            test_rewards.append(episode_reward)
            test_profits.append(info.get('total_profit', 0))
            test_balances.append(info.get('balance', 0))
            
            if hasattr(test_env, 'trade_history'):
                all_trades.extend(test_env.trade_history)
            
            print(f"Episode {ep+1}/{n_episodes}: Reward={episode_reward:.2f}, Profit=${info.get('total_profit', 0):.2f}")
        
        # 통계 계산
        results = {
            'avg_reward': np.mean(test_rewards),
            'std_reward': np.std(test_rewards),
            'avg_profit': np.mean(test_profits),
            'std_profit': np.std(test_profits),
            'avg_balance': np.mean(test_balances),
            'win_rate': np.mean([p > 0 for p in test_profits]),
            'total_trades': len(all_trades)
        }
        
        print("\n" + "="*60)
        print("📊 Test Results:")
        print(f"   Avg Reward: {results['avg_reward']:.2f} ± {results['std_reward']:.2f}")
        print(f"   Avg Profit: ${results['avg_profit']:.2f} ± ${results['std_profit']:.2f}")
        print(f"   Win Rate: {results['win_rate']*100:.2f}%")
        print(f"   Total Trades: {results['total_trades']}")
        print("="*60)
        
        # 테스트 결과 저장
        test_stats_path = os.path.join(self.config.training.checkpoint_dir, 'test_results.json')
        with open(test_stats_path, 'w') as f:
            json.dump(results, f, indent=4)
        
        return results


def create_trainer_from_data(
    train_data: np.ndarray,
    val_data: Optional[np.ndarray] = None,
    config: Optional[RLConfig] = None
) -> RLTrainer:
    """
    데이터로부터 트레이너 생성 (편의 함수)
    
    Args:
        train_data: 학습 데이터
        val_data: 검증 데이터
        config: 설정 (None이면 기본 설정 사용)
    
    Returns:
        RLTrainer 객체
    """
    if config is None:
        from .config import RLConfig
        config = RLConfig.default()
    
    # 환경 생성
    train_env = TradingEnvironment(
        data=train_data,
        initial_balance=config.env.initial_balance,
        transaction_fee=config.env.transaction_fee,
        window_size=config.env.window_size,
        max_position=config.env.max_position
    )
    
    val_env = None
    if val_data is not None:
        val_env = TradingEnvironment(
            data=val_data,
            initial_balance=config.env.initial_balance,
            transaction_fee=config.env.transaction_fee,
            window_size=config.env.window_size,
            max_position=config.env.max_position
        )
    
    # 상태 차원 계산
    state_dim = train_env.observation_space.shape[0] * train_env.observation_space.shape[1]
    action_dim = train_env.action_space.shape[0]
    
    # 에이전트 생성
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
        max_grad_norm=config.ppo.max_grad_norm,
        use_recurrent=config.network.use_recurrent,
        device=config.training.device
    )
    
    # 트레이너 생성
    trainer = RLTrainer(
        env=train_env,
        agent=agent,
        config=config,
        val_env=val_env
    )
    
    return trainer