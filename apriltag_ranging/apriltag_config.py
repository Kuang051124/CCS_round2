"""
AprilTag 共享参数 — 加载/保存/默认值
"""
import os

CONFIG_FILE = "/root/apriltag_config.txt"

DEFAULT_TAG_FAMILY   = "TAG36H11"
DEFAULT_TAG_SIZE_MM  = 80.0
DEFAULT_FOCAL_LENGTH = 100.0
DEFAULT_MIN_AREA     = 100
DEFAULT_DISTORT_K1   = 0.0
DEFAULT_IMG_SIZE     = 320
DEFAULT_CAL_DIST_MM  = 300.0     # 一键标定的已知距离 (mm)
DEFAULT_MIN_CONF     = 20         # 最小置信度 (decision_margin), 0=不过滤
DEFAULT_OUTPUT_MODE  = "nearest"

tag_family   = DEFAULT_TAG_FAMILY
tag_size_mm  = DEFAULT_TAG_SIZE_MM
focal_length = DEFAULT_FOCAL_LENGTH
min_area     = DEFAULT_MIN_AREA
distort_k1   = DEFAULT_DISTORT_K1
img_size     = DEFAULT_IMG_SIZE
cal_dist_mm  = DEFAULT_CAL_DIST_MM
min_conf     = DEFAULT_MIN_CONF
output_mode  = DEFAULT_OUTPUT_MODE

_AVAILABLE_FAMILIES = [
    "TAG36H11", "TAG36H10", "TAG25H9", "TAG25H7", "TAG16H5"
]


def load(path=CONFIG_FILE):
    if not os.path.exists(path):
        print("[Config] No config file, using defaults")
        return False
    try:
        with open(path, "r") as f:
            raw = f.read().strip()
        for line in raw.split("\n"):
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            if "=" not in line:
                continue
            key, val = line.split("=", 1)
            key, val = key.strip(), val.strip()

            global tag_family, tag_size_mm, focal_length
            global min_area, distort_k1, img_size, cal_dist_mm, min_conf, output_mode

            if key == "tag_family" and val in _AVAILABLE_FAMILIES:
                tag_family = val
            elif key == "tag_size_mm":
                tag_size_mm = float(val)
            elif key == "focal_length":
                focal_length = float(val)
            elif key == "min_area":
                min_area = int(val)
            elif key == "distort_k1":
                distort_k1 = float(val)
            elif key == "img_size":
                img_size = int(val)
            elif key == "cal_dist_mm":
                cal_dist_mm = float(val)
            elif key == "min_conf":
                min_conf = float(val)
            elif key == "output_mode":
                output_mode = val if val in ("nearest", "all") else "nearest"

        print(f"[Config] Loaded from {path}")
        return True
    except Exception as e:
        print(f"[Config] Load failed: {e}")
        return False


def save(path=CONFIG_FILE):
    try:
        lines = [
            f"# AprilTag Ranging Config",
            f"tag_family={tag_family}",
            f"tag_size_mm={tag_size_mm}",
            f"focal_length={focal_length}",
            f"min_area={min_area}",
            f"distort_k1={distort_k1}",
            f"img_size={img_size}",
            f"cal_dist_mm={cal_dist_mm}",
            f"min_conf={min_conf}",
            f"output_mode={output_mode}",
        ]
        with open(path, "w") as f:
            f.write("\n".join(lines))
        print(f"[Config] Saved to {path}")
        return True
    except Exception as e:
        print(f"[Config] Save failed: {e}")
        return False


def reset_defaults():
    global tag_family, tag_size_mm, focal_length, min_area, distort_k1, img_size, cal_dist_mm, min_conf, output_mode
    tag_family   = DEFAULT_TAG_FAMILY
    tag_size_mm  = DEFAULT_TAG_SIZE_MM
    focal_length = DEFAULT_FOCAL_LENGTH
    min_area     = DEFAULT_MIN_AREA
    distort_k1   = DEFAULT_DISTORT_K1
    img_size     = DEFAULT_IMG_SIZE
    cal_dist_mm  = DEFAULT_CAL_DIST_MM
    min_conf     = DEFAULT_MIN_CONF
    output_mode  = DEFAULT_OUTPUT_MODE
    print("[Config] Reset to defaults")


if __name__ == "__main__":
    print("=== AprilTag Config ===")
    print(f"  tag_family   = {tag_family}")
    print(f"  tag_size_mm  = {tag_size_mm}")
    print(f"  focal_length = {focal_length}")
    print(f"  min_area     = {min_area}")
    print(f"  distort_k1   = {distort_k1}")
    print(f"  img_size     = {img_size}")
    print(f"  output_mode  = {output_mode}")
    print(f"  config_file  = {CONFIG_FILE}")
    print()
    save()
    print("  Config updated.")
