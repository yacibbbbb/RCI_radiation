#!/usr/bin/env python3
import csv
import os
import sys
import math
import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple

from pathlib import Path

# ================== .csv 파일 경로 및 파일명 설정 부분 ==================
WORLD_NAME = 'single_source'

RAD_DIR = Path(__file__).resolve().parents[1]
BASE_DIR = os.path.join(RAD_DIR, 'data/cumulation', WORLD_NAME)

NO_AVOID_CSV = os.path.join(BASE_DIR, "no_avoid_vel10.csv")
AVOID_CSV    = os.path.join(BASE_DIR, "avoid_thres30_60deg_ver4.csv")
# ==========================================================

# ======================== 색상 고정 =========================
COLOR_NO  = "#1f77b4"  # 파랑
COLOR_AVO = "#ff7f0e"  # 주황
# ==========================================================

def load_csv(path: str):
    """CSV에서 t,intensity,x,y 컬럼을 읽어 numpy 배열로 반환. NaN/빈칸은 건너뜀."""
    t_list, I_list, x_list, y_list = [], [], [], []
    with open(path, 'r') as f:
        rd = csv.DictReader(f)
        for row in rd:
            try:
                t = float(row['t_sec'])
                I = float(row['intensity'])
                x = float(row['x']) if row.get('x', '') not in ('', 'nan', 'NaN') else np.nan
                y = float(row['y']) if row.get('y', '') not in ('', 'nan', 'NaN') else np.nan
                # 위치는 궤적용, intensity 곡선은 위치 없어도 됨. t,I는 필수.
                if not math.isfinite(t) or not math.isfinite(I):
                    continue
                t_list.append(t); I_list.append(I)
                x_list.append(x); y_list.append(y)
            except Exception:
                continue
    if len(t_list) == 0:
        return np.array([]), np.array([]), np.array([]), np.array([])
    t = np.array(t_list); I = np.array(I_list)
    x = np.array(x_list); y = np.array(y_list)
    # 시간 정렬(무질서 대비)
    idx = np.argsort(t)
    return t[idx], I[idx], x[idx], y[idx]

def time_weighted_mean(t: np.ndarray, I: np.ndarray) -> float:
    """시간가중 평균: sum(I*dt)/sum(dt) (직사각형 규칙, 마지막 구간 제외)"""
    if len(t) < 2:
        return float(I.mean()) if len(I) > 0 else float('nan')
    dt = np.diff(t)
    dt[dt < 0] = 0.0
    num = np.sum(I[:-1] * dt)
    den = np.sum(dt)
    return (num / den) if den > 0 else float('nan')

def cumulative_index(t: np.ndarray, I: np.ndarray) -> Tuple[float, np.ndarray, np.ndarray]:
    """누적지표 S = ∑ I*dt (직사각형 규칙). 반환: 최종 S, 누적시계열 c(t), 정규화된 시간축 τ=t-t0"""
    if len(t) == 0:
        return 0.0, np.array([]), np.array([])
    tau = t - t[0]
    if len(t) == 1:
        return 0.0, np.array([0.0]), tau
    dt = np.diff(t)
    dt[dt < 0] = 0.0
    partial = I[:-1] * dt
    c = np.concatenate([[0.0], np.cumsum(partial)])
    return float(c[-1]), c, tau

def plot_intensity_panel(t0, I0, t1, I1, label0='no_avoid', label1='avoid'):
    """
    화면 1: Intensity vs Time + 평균선(시간가중 평균).
    - 스타일: no_avoid=점선(--), avoid=실선(-)
    - 각 곡선의 시간 범위에서만 평균선 표시
    """
    fig, ax = plt.subplots()
    # 시간 정규화
    tau0 = t0 - t0[0] if len(t0) > 0 else np.array([])
    tau1 = t1 - t1[0] if len(t1) > 0 else np.array([])

    # 곡선
    if len(t0) > 0:
        ax.plot(tau0, I0, linestyle='-', label=label0, color=COLOR_NO)
    if len(t1) > 0:
        ax.plot(tau1, I1, linestyle='-', label=label1, color=COLOR_AVO)

    # 평균선 (시간가중)
    m0 = time_weighted_mean(t0, I0) if len(t0) > 1 else (I0.mean() if len(I0) else np.nan)
    m1 = time_weighted_mean(t1, I1) if len(t1) > 1 else (I1.mean() if len(I1) else np.nan)

    if len(t0) > 0 and math.isfinite(m0):
        xmax0 = tau0[-1] if len(t0) else 0.0
        ax.hlines(m0, xmin=0.0, xmax=xmax0,
                  linestyles='--', linewidth=1.5, label=f'{label0} mean', color=COLOR_NO)
        # 평균 값 텍스트 추가
        ax.text(0.1, m0, f'{m0:.2f}', color=COLOR_NO,
                va='bottom', ha='left', fontsize=14, fontweight='bold')
    if len(t1) > 0 and math.isfinite(m1):
        xmax1 = tau1[-1] if len(t1) else 0.0
        ax.hlines(m1, xmin=0.0, xmax=xmax1,
                  linestyles='--', linewidth=1.5, label=f'{label1} mean', color=COLOR_AVO)
        # 평균 값 텍스트 추가
        ax.text(0.1, m1, f'{m1:.2f}', color=COLOR_AVO,
                va='bottom', ha='left', fontsize=14, fontweight='bold')

    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Intensity (dimensionless)')
    ax.set_title('Time-Intensity (with time-weighted means)')
    ax.grid(True)
    ax.legend()
    return fig, ax

def plot_trajectory_panel(x0, y0, x1, y1,
                          xlim=(-6.5, 6.5), ylim=(-5.5, 5.5),
                          start=(-6.0, 0.0), goal=(6.0, 0.0), source=(0.0, 0.0),
                          label0='no_avoid', label1='avoid', radii=(1.5, 3.0, 4.5, 6.0)):
    """
    화면 2: 궤적(x–y)
    - 범위/표식 요구조건 반영
    - 스타일: no_avoid=점선, avoid=실선
    - 시작: 노란 원 + 검은 테두리 / 목표: 노란 별 + 검은 테두리 / 소스: 빨간 X
    """
    fig, ax = plt.subplots()
 
    # source 중심의 contour line
    import matplotlib.patches as patches
    for r in radii:
        circ = patches.Circle(source, r, fill=False, lw=2.0, ec='red', alpha=0.2)
        ax.add_patch(circ)

    # 궤적(NaN 제거)
    if len(x0) > 1:
        m = np.isfinite(x0) & np.isfinite(y0)
        ax.plot(x0[m], y0[m], linestyle='--', label=label0)
    if len(x1) > 1:
        m = np.isfinite(x1) & np.isfinite(y1)
        ax.plot(x1[m], y1[m], linestyle='-', label=label1)

    # 시작/목표/소스 표식
    ax.scatter(start[0], start[1], marker='o', s=100, facecolor='yellow', edgecolor='black', zorder=5, label='start')
    ax.scatter(goal[0],  goal[1],  marker='*', s=180, facecolor='yellow', edgecolor='black', zorder=6, label='goal')
    ax.scatter(source[0], source[1], marker='x', s=100, color='red', linewidths=2, zorder=7, label='source')

    ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
    ax.set_title('Trajectory')
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlim(*xlim); ax.set_ylim(*ylim)
    ax.grid(True); ax.legend()
    return fig, ax


def plot_cumulative_panel(t0, I0, t1, I1):
    """화면 3: 누적지표(∑ I·Δt) vs Time (색상 고정)"""
    S0, C0, tau0 = cumulative_index(t0, I0)
    S1, C1, tau1 = cumulative_index(t1, I1)

    fig, ax = plt.subplots()
    if len(C0) > 0:
        ax.plot(tau0, C0, linestyle='--', color=COLOR_NO, label=f'no_avoid (final={S0:.3f})')
    if len(C1) > 0:
        ax.plot(tau1, C1, linestyle='-',  color=COLOR_AVO, label=f'avoid (final={S1:.3f})')

    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Cumulative Index (∑ I·Δt)')
    ax.set_title('Time-Cumulative Exposure')
    ax.grid(True)
    ax.legend()
    return fig, ax

def main():
    # 1) CSV 로드
    t0,I0,x0,y0 = load_csv(NO_AVOID_CSV)
    t1,I1,x1,y1 = load_csv(AVOID_CSV)

    if len(t0) == 0 and len(t1) == 0:
        print('두 CSV 모두 비었습니다.')
        sys.exit(0)

    # 2) 누적지표 계산 + 콘솔 출력
    S0, C0, tau0 = cumulative_index(t0, I0)
    S1, C1, tau1 = cumulative_index(t1, I1)
    print(f'No-avoid cumulative index (∑ I·Δt): {S0:.3f}')
    print(f'Avoid   cumulative index (∑ I·Δt): {S1:.3f}')
    if S0 > 1e-12:
        print(f'Reduction: {(1.0 - S1/S0)*100.0:.2f}%')

    # 3) 화면1: Intensity vs Time (+ 평균선)
    fig1, ax1 = plot_intensity_panel(t0, I0, t1, I1, label0='no_avoid', label1='avoid')

    # # 누적곡선도 참고로 같이 보고 싶으면(옵션): 아래 주석 해제
    # if len(C0) > 0: ax1_2 = ax1.twinx(); ax1_2.plot(tau0, C0, ':', alpha=0.5, label='no_avoid cumulative')
    # if len(C1) > 0: ax1_2.plot(tau1, C1, '-', alpha=0.3, label='avoid cumulative')
    # if len(C0) > 0 or len(C1) > 0:
    #     ax1_2.set_ylabel('Cumulative Index'); ax1_2.legend(loc='lower right')

    # 4) 화면2: 궤적
    fig2, ax2 = plot_trajectory_panel(
        x0, y0, x1, y1,
        xlim=(-6.5, 6.5), ylim=(-5.5, 5.5),
        start=(-6.0, 0.0), goal=(6.0, 0.0), source=(0.0, 0.0),
        label0='no_avoid', label1='avoid',
        radii=(0.1, 0.3, 0.35, 0.45, 0.6, 1.05, 1.414, 2, 3.5)
    )

    # 5) 화면 3: 누적지표만
    plot_cumulative_panel(t0, I0, t1, I1)

    plt.show()

if __name__ == '__main__':
    main()
