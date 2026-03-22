#!/usr/bin/env python3
import cv2
import yaml
import numpy as np
from pathlib import Path
import os

from pathlib import Path

def load_map(yaml_path):
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
    image_path = Path(yaml_path).parent / data["image"]
    img = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    assert img is not None, f"이미지 로딩 실패: {image_path}"
    return data, img

def world_bounds(yaml_data, img):
    res = yaml_data["resolution"]
    ox, oy = yaml_data["origin"][:2]
    h, w = img.shape
    x_min = ox
    y_min = oy
    x_max = ox + w * res
    y_max = oy + h * res
    return x_min, y_min, x_max, y_max

def merge_maps(map_yaml_path, rad_yaml_path, output_dir, output_basename):
    map_yaml, map_img = load_map(map_yaml_path)
    rad_yaml, rad_img = load_map(rad_yaml_path)

    res = map_yaml["resolution"]
    assert abs(res - rad_yaml["resolution"]) < 1e-6, "해상도 불일치!"

    # 전체 월드 좌표 bounding box 계산
    mx1, my1, mx2, my2 = world_bounds(map_yaml, map_img)
    rx1, ry1, rx2, ry2 = world_bounds(rad_yaml, rad_img)

    wx1, wy1 = min(mx1, rx1), min(my1, ry1)
    wx2, wy2 = max(mx2, rx2), max(my2, ry2)

    width = int(np.ceil((wx2 - wx1) / res))
    height = int(np.ceil((wy2 - wy1) / res))

    canvas = np.full((height, width), 205, dtype=np.uint8)  # unknown

    def paste(img, yaml_data):
        ox, oy = yaml_data["origin"][:2]
        h, w = img.shape
        offset_x = int(round((ox - wx1) / res))
        offset_y = int(round((oy - wy1) / res))
        for y in range(h):
            for x in range(w):
                val = img[y, x]
                if val == 205:
                    continue
                cy = offset_y + y
                cx = offset_x + x
                if 0 <= cy < height and 0 <= cx < width:
                    cur = canvas[cy, cx]
                    if cur == 205:
                        canvas[cy, cx] = val
                    else:
                        canvas[cy, cx] = min(cur, val)

    paste(map_img, map_yaml)
    paste(rad_img, rad_yaml)

    # 저장
    os.makedirs(output_dir, exist_ok=True)
    out_pgm = os.path.join(output_dir, output_basename + ".pgm")
    out_yaml = os.path.join(output_dir, output_basename + ".yaml")
    cv2.imwrite(out_pgm, canvas)

    merged_yaml = {
        "image": Path(out_pgm).name,
        "resolution": res,
        "origin": [wx1, wy1, 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.25,
        "mode": "trinary"
    }
    with open(out_yaml, 'w') as f:
        yaml.dump(merged_yaml, f)

    print(f"[✓] 병합 완료: {out_pgm}, {out_yaml}")

if __name__ == "__main__":

    radiation_sim_dir = Path(__file__).resolve().parents[1]

    merge_maps(
        map_yaml_path=os.path.join(radiation_sim_dir, 'map/obstacle', 'reactor_room.yaml'),
        rad_yaml_path=os.path.join(radiation_sim_dir, 'map/radiation_static', 'radiation_static_map_resized.yaml'),
        output_dir=os.path.join(radiation_sim_dir, 'map/merged'),
        output_basename="merged_map"
    )
