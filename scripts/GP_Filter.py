#!/usr/bin/env python3
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import csv
import os
import yaml

from pathlib import Path

from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, ConstantKernel as C
from matplotlib.colors import ListedColormap
from scipy.ndimage import gaussian_filter  # smoothing 용

# --------------------------------------------------
#  
# 기본적으로, 장애물과 방사능원에 대한 정보(csv)는 data/(월드명) 내부 폴더에서 불러옴.
# radiation_map_generator.cpp는 주기적으로 data/current_map 폴더에 intensity.csv를 저장하기 때문에 1번 메뉴는 data에서 csv_file을 찾음.
# 1번 메뉴를 통해 CSV파일의 output이 data/current_map 폴더 내부에 생성되었다면, 다시 스크립트를 실행하기 전 해당 월드 폴더에 이동시킬 것.
# 2번 메뉴를 통해 존재하던 GP가 실행된 CSV파일을 통해 plot만 하고자 한다면, 해당 월드 폴더 내부에 있는 CSV파일을 불러옴.
# 3번 메뉴를 통해 sweeper를 통해 얻은 entire_intensity.csv를 plot만 함.
#
# -------------------- 설정 부분 --------------------

radiation_sim_dir = Path(__file__).resolve().parents[1]

# data/(월드 명)
pkg_name = "reactor_room"
route_start = os.path.join(radiation_sim_dir, 'data') + '/'

# CSV 파일 경로(Intensity) >> current_map에서 불러옴
csv_file = route_start + "current_map/intensity.csv"

# 소스 & 장애물 정보 >> Plot용
obstacle_file = route_start + pkg_name + "/obstacles.csv"
source_file = route_start + pkg_name + "/sources.csv"

# 결과 저장 경로 >> current_map에 저장함
output_csv = route_start + "current_map/intensity_gp.csv"

# 이미 GP가 실행된 결과 CSV 경로
existing_gp_csv = route_start + pkg_name + "/intensity_gp.csv"

# sweeper를 통해 얻은 CSV 경로
entire_intensity_csv = route_start + pkg_name + "/entire_intensity.csv"

# .yaml로부터 map data 불러오기
# [ path 확인할 것!! 매우 중요! ]
yaml_path = os.path.join(radiation_sim_dir, 'config', pkg_name, 'map_data.yaml')
with open(yaml_path, 'r') as f:
    map_data = yaml.safe_load(f)['map_data']

# radiation_map_generator.cpp 의 max_intensity 값과 맞춰야 함!
# 이제 radiation_map_generator.cpp에서도 .yaml 기반으로 하기 때문에 신경쓰지 않아도 됨.
max_intensity = map_data['max_intensity']

# 맵 크기 및 origin (launch 파일의 파라미터와 동일하게 하면 됨)
map_width = map_data['map_width']
map_height = map_data['map_height']
map_origin_x = map_data['map_origin_x']
map_origin_y = map_data['map_origin_y']

# 해상도 (낮을수록 더 부드럽긴 하나, 복잡도 증가 -> cost 증가)
# 너무 낮으면 제대로 된 Map 생성 X
map_resolution = map_data['map_resolution']

# --------------------------------------------------

def print_info():
    print("--------------------------------------------")
    print("<info>")
    print(f"world_name: {pkg_name}")
    print(f"max_intensity: {max_intensity}")
    print(f"map_width: {map_width}")
    print(f"map_height: {map_height}")
    print(f"map_origin_x: {map_origin_x}")
    print(f"map_origin_y: {map_origin_y}")
    print(f"map_resolution: {map_resolution}")
    print("--------------------------------------------")

def load_csv(file_path):
    data = []
    with open(file_path, newline='') as csvfile:
        reader = csv.reader(csvfile)
        next(reader)
        for row in reader:
            x, y, intensity = float(row[0]), float(row[1]), float(row[2])
            if intensity >= 0:
                data.append((x, y, intensity))
    return np.array(data)

def load_obstacles():
    obstacles = []
    try:
        obs_df = pd.read_csv(obstacle_file)
        for _, row in obs_df.iterrows():
            obstacles.append({'name': row['name'], 'x': row['x'], 'y': row['y'],
                              'size_x': row['size_x'], 'size_y': row['size_y']})
        print("장애물 CSV 읽기 성공.")
    except Exception as e:
        print(f"장애물 CSV 읽기 실패: {e}")
    return obstacles

def load_sources():
    sources = []
    try:
        src_df = pd.read_csv(source_file)
        for _, row in src_df.iterrows():
            sources.append({'name': row['name'], 'x': row['x'], 'y': row['y'], 'radius': row['radius']})
        print("방사능원 CSV 읽기 성공.")
    except Exception as e:
        print(f"방사능원 CSV 읽기 실패: {e}")
    return sources

def plot_result(xx, yy, intensity_grid, obstacles, sources, title="GP Radiation Map"):
    normalized_grid = np.clip(intensity_grid / max_intensity, 0, 1.0)

    # 256 색상 Interpolation
    colors = [
        (0.0, 0.0, 1.0),   # 파랑
        (0.0, 1.0, 1.0),   # 청록
        (0.0, 1.0, 0.0),   # 초록
        (1.0, 1.0, 0.0),   # 노랑
        (1.0, 0.0, 0.0)    # 빨강
    ]

    cmap = ListedColormap([
        (
            colors[0][0] + (colors[1][0] - colors[0][0]) * i / 64,
            colors[0][1] + (colors[1][1] - colors[0][1]) * i / 64,
            colors[0][2] + (colors[1][2] - colors[0][2]) * i / 64
        ) if i < 64 else (
            colors[1][0] + (colors[2][0] - colors[1][0]) * (i - 64) / 64,
            colors[1][1] + (colors[2][1] - colors[1][1]) * (i - 64) / 64,
            colors[1][2] + (colors[2][2] - colors[1][2]) * (i - 64) / 64
        ) if i < 128 else (
            colors[2][0] + (colors[3][0] - colors[2][0]) * (i - 128) / 64,
            colors[2][1] + (colors[3][1] - colors[2][1]) * (i - 128) / 64,
            colors[2][2] + (colors[3][2] - colors[2][2]) * (i - 128) / 64
        ) if i < 192 else (
            colors[3][0] + (colors[4][0] - colors[3][0]) * (i - 192) / 63,
            colors[3][1] + (colors[4][1] - colors[3][1]) * (i - 192) / 63,
            colors[3][2] + (colors[4][2] - colors[3][2]) * (i - 192) / 63
        )
        for i in range(256)
    ])

    # 시각화
    plt.figure(figsize=(8, 8))

    plt.imshow(normalized_grid, origin='lower',
               extent=[map_origin_x, map_origin_x + map_width,
                       map_origin_y, map_origin_y + map_height],
               cmap=cmap, interpolation='bicubic')
    
    plt.colorbar(label='Normalized Radiation Intensity')

    # 장애물 시각화
    for obs in obstacles:
        if obs['name'] == "radiation_obstacle_walls_boratedConcrete" : continue
        if obs['name'] == "radiation_obstacle_TankShielding_boratedConcrete" : continue
        rect = plt.Rectangle(
            (obs['x'] - obs['size_x'] / 2, obs['y'] - obs['size_y'] / 2),
            obs['size_x'], obs['size_y'],
            linewidth=1, edgecolor='black', facecolor='gray', alpha=1.0
        )
        plt.gca().add_patch(rect)

    # Radiation source 시각화
    for src in sources:
        circle = plt.Circle(
            (src['x'], src['y']), src['radius'],
            facecolor='yellow', alpha=1.0,
            edgecolor='black', linewidth=2
        )
        plt.gca().add_patch(circle)

    plt.title(title)
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.grid(True)
    plt.show()

def run_gp():
    data = load_csv(csv_file)
    X = data[:, :2]
    y = data[:, 2]
    print(f"총 {len(X)}개의 측정 포인트 사용.")

    # length_scale이 클수록 smoothness 증가, 너무 크면 디테일이 사라짐
    kernel = C(1.0) * RBF(length_scale=1.0)
    # alpha 값이 클수록 오차 허용 범위 증가 > 연산 가능성 증가
    # alpha 값이 너무 작으면 제대로 된 연산 X
    gp = GaussianProcessRegressor(kernel=kernel, n_restarts_optimizer=10, alpha=0.5)

    # GP fitting
    gp.fit(X, y)
    print("Gaussian Process fitting 완료.")

    # 전체 맵의 Grid Points 준비
    xx, yy = np.meshgrid(
        np.arange(map_origin_x, map_origin_x + map_width, map_resolution),
        np.arange(map_origin_y, map_origin_y + map_height, map_resolution)
    )
    grid_points = np.c_[xx.ravel(), yy.ravel()]

    # GP로 전체 맵 예측
    y_pred, sigma = gp.predict(grid_points, return_std=True)

    # 결과 Grid로 변환
    intensity_grid = y_pred.reshape(xx.shape)
    # variance_grid = sigma.reshape(xx.shape) # potential error

    # CSV로 예측 결과 저장
    with open(output_csv, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["x", "y", "predicted_intensity"])
        for xi, yi, intensity in zip(grid_points[:, 0], grid_points[:, 1], y_pred):
            writer.writerow([xi, yi, intensity])

    print(f"Gaussian Process 예측 결과를 {output_csv} 에 저장했습니다.")
    return xx, yy, intensity_grid

def plot_existing_gp():
    try:
        gp_data = pd.read_csv(existing_gp_csv)
        xx, yy = np.meshgrid(
            np.arange(map_origin_x, map_origin_x + map_width, map_resolution),
            np.arange(map_origin_y, map_origin_y + map_height, map_resolution)
        )
        intensity_grid = gp_data['predicted_intensity'].values.reshape(xx.shape)
        return xx, yy, intensity_grid
    except Exception as e:
        print(f"GP csv 파일 로드 실패: {e}")
        return None, None, None
    
def plot_entire_intensity_smoothed(sigma=1.0):
    try:
        grid = np.loadtxt(entire_intensity_csv, delimiter=',')
    except Exception as e:
        print(f"entire_intensity.csv 로드 실패: {e}")
        return
    
    smoothed = gaussian_filter(grid, sigma=sigma)
    obstacles = load_obstacles()
    sources = load_sources()

    xx, yy = np.meshgrid(
        np.arange(map_origin_x, map_origin_x + map_width, map_resolution),
        np.arange(map_origin_y, map_origin_y + map_height, map_resolution)
    )

    plot_result(xx, yy, smoothed, obstacles, sources,
                title=f"Smoothed Entire Intensity (σ={sigma})")

def main():
    while True:
        print_info()
        print("<menu>")
        print("1. intensity.csv > GP > plot")
        print("2. intensity_gp.csv > plot")
        print("3. entire_intensity.csv > plot")
        print("4. quit")

        choice = input("선택 (1~4): ").strip()

        # 탐사 직후 바로 GP 실행 후 plot
        if choice == "1":
            xx, yy, intensity_grid = run_gp()
            obstacles = load_obstacles()
            sources = load_sources()
            plot_result(xx, yy, intensity_grid, obstacles, sources)

        # 이미 GP를 실행한 CSV 데이터가 있는 경우
        elif choice == "2":
            xx, yy, intensity_grid = plot_existing_gp()
            if xx is not None:
                obstacles = load_obstacles()
                sources = load_sources()
                plot_result(xx, yy, intensity_grid, obstacles, sources)
        
        elif choice == "3":
            try:
                sg = input("sigma 값 (기본=1.0): ").strip()
                sigma = float(sg) if sg else 1.0
            except:
                sigma = 1.0
            plot_entire_intensity_smoothed(sigma)

        elif choice == "4":
            print("프로그램 종료.")
            break

        else:
            print("잘못된 입력입니다. 다시 선택하세요.")

if __name__ == "__main__":
    if not os.path.exists(route_start):
        print(f"경로 {route_start} 가 존재하지 않습니다. 확인해주세요.")
    else:
        main()
