#!/usr/bin/env python3
import pandas as pd
import numpy as np
import imageio
import yaml
import os

from pathlib import Path

# 0. world name, 출력 파일 이름 설정
world_name = 'reactor_room'
output_file_name = 'radiation_static_map_ver2'

# 1. 경로 설정
base_dir = Path(__file__).resolve().parents[4]
radiation_sim_dir = base_dir + "/src/radiation_simulation"

csv_path = os.path.join(radiation_sim_dir, 'data', world_name, 'intensity_gp.csv')
map_data_yaml = os.path.join(radiation_sim_dir, 'config', world_name, '/map_data.yaml')
output_dir = os.path.join(radiation_sim_dir, 'map/radiation_static')

# 2. map_data.yaml에서 메타데이터 읽기
with open(map_data_yaml, 'r') as f:
    map_data = yaml.safe_load(f)['map_data']

resolution = map_data['map_resolution']
origin = [map_data['map_origin_x'], map_data['map_origin_y'], 0.0]
width = int(map_data['map_width'] / resolution)
height = int(map_data['map_height'] / resolution)
max_intensity = map_data['max_intensity']

# 3. CSV 읽기
df = pd.read_csv(csv_path)

# 4. 그리드 초기화
grid = np.zeros((height, width), dtype=np.uint8)

# 5. 좌표 → 그리드 인덱스 변환 및 강도 매핑
for _, row in df.iterrows():
    x_idx = int((row['x'] - origin[0]) / resolution)
    y_idx = int((row['y'] - origin[1]) / resolution)
    if 0 <= x_idx < width and 0 <= y_idx < height:
        intensity = min(row['predicted_intensity'], max_intensity)

        # 방사능 높은 곳이 하얀색으로 되어 반전시켜야 함.
        cost = 255 - int(255 * (intensity / max_intensity))
        
        # 안전하게 cost를 0과 255 사이로 clamping
        cost = max(0, min(cost, 255))
        grid[height - y_idx - 1, x_idx] = cost

# 6. 출력 파일 경로
dot_pgm = '.pgm'
dot_yaml = '.yaml'

output_pgm = output_file_name + dot_pgm
output_yaml = output_file_name + dot_yaml

pgm_path = os.path.join(output_dir, output_pgm)
yaml_path = os.path.join(output_dir, output_yaml)

# 7. PGM 저장
imageio.imwrite(pgm_path, grid)

# 8. YAML 저장
with open(yaml_path, 'w') as f:
    # negate: 이미지 색상 반전 여부 (0: 검정이 장애물, 1: 흰색이 장애물)
    # occupied_thresh: normalized value(intensity)가 값을 초과하면 장애물로 간주
    # free_thresh: 이 값 미만이면 자유 공간(free space)으로 간주됨

    yaml.dump({
        'image': os.path.basename(pgm_path),
        'resolution': resolution,
        'origin': origin,
        'negate': 0,
        'occupied_thresh': 0.8,
        'free_thresh': 0.1
    }, f)

print("PGM & YAML 파일이 생성되었습니다.")
