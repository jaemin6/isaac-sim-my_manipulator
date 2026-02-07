import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.distributions import Normal
from typing import Tuple, List, Dict
import os

# 1. 아키텍처: MLP 기반 Actor-Critic
class ActorCritic(nn.Module):
    def __init__(self, ...): # 레이어 정의 (Feature, Actor, Critic)
    def _initialize_weights(self): # 가중치 초기화
    def forward(self, state): # 순전파 (Mean, Log_std, Value 반환)
    def get_action(self, state, deterministic): # 행동 샘플링 및 Log_prob 계산
    def evaluate_actions(self, state, action): # 학습용 Log_prob, Entropy 계산

# 2. 알고리즘: PPO 에이전트
class PPOAgent:
    def __init__(self, ...): # 하이퍼파라미터 및 네트워크 초기화
    def select_action(self, state, deterministic): # 환경에 적용할 행동 선택
    def compute_gae(self, rewards, values, dones, next_value): # Advantage 계산 (핵심)
    def update(self, ...): # 에폭/배치별 PPO 손실 함수 계산 및 최적화
        # 1. Advantage 정규화
        # 2. Policy Loss (Clipping) + Value Loss + Entropy 계산
        # 3. 역전파 및 그래디언트 클리핑
    def save/load(self, filepath): # 모델 영구 저장 및 복구

# 3. 아키텍처: LSTM 기반 Recurrent Actor-Critic
class RecurrentActorCritic(nn.Module):
    def __init__(self, ...): # LSTM 및 헤드 정의
    def forward(self, state, hidden_state): # 순차 데이터 처리 및 Hidden state 갱신