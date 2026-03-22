#!/usr/bin/env python3
import cv2
import numpy as np
import yaml
import os
from pathlib import Path

# map.pgm과 .yaml파일은 resolution이 0.05인 상태로 측정되지만,
# radiation을 0.05로 측정할 경우 너무 많은 점(cell)을 지나야 한다.
# 따라서 resolution을 0.25로 측정한 후, 이 스크립트를 통해 선형 보간하여 0.05로 측정한 것처럼 resize 한다.

def load_yaml(yaml_path):
    with open(yaml_path, 'r') as f:
        return yaml.safe_load(f)

def resize_radiation_map(input_yaml_path, output_dir, output_pgm_name, output_yaml_name, target_resolution):
    # 원본 YAML 읽기
    yaml_data = load_yaml(input_yaml_path)
    input_img_path = str(Path(input_yaml_path).parent / yaml_data["image"])
    original_resolution = float(yaml_data["resolution"])

    # 원본 PGM 불러오기
    img = cv2.imread(input_img_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise RuntimeError(f"이미지 불러오기 실패: {input_img_path}")

    # 보간 비율 계산 및 크기 설정
    scale = original_resolution / target_resolution
    new_size = (int(img.shape[1] * scale), int(img.shape[0] * scale))

    resized_img = cv2.resize(img, new_size, interpolation=cv2.INTER_LINEAR)

    # 결과 저장 경로
    out_pgm_path = os.path.join(output_dir, output_pgm_name)
    out_yaml_path = os.path.join(output_dir, output_yaml_name)

    # 새 PGM 이미지 저장
    cv2.imwrite(out_pgm_path, resized_img)

    # YAML 수정 및 저장
    new_yaml = yaml_data.copy()
    new_yaml["image"] = output_pgm_name
    new_yaml["resolution"] = target_resolution

    with open(out_yaml_path, 'w') as f:
        yaml.dump(new_yaml, f)

    print(f"[✓] 보간 완료: {out_pgm_path}, {out_yaml_path}")

if __name__ == "__main__":
    radiation_sim_dir = Path(__file__).resolve().parents[1]

    resize_radiation_map(
        input_yaml_path=os.path.join(radiation_sim_dir, 'map/radiation_static', 'radiation_static_map.yaml'),
        output_dir=os.path.join(radiation_sim_dir, 'map/radiation_static'),
        output_pgm_name="radiation_static_map_resized2.pgm",
        output_yaml_name="radiation_static_map_resized2.yaml",
        target_resolution=0.05
    )
