# AprilTag 相机焦距标定工具
#
# 步骤:
#   1. 修改下方 TAG_SIZE_MM / KNOWN_DIST_MM
#   2. 将标签放于相机正前方已知距离处
#   3. 在 MaixCAM 上运行: python calibrate.py
#   4. 按键采集3次样本 → 自动计算焦距
#   5. 标定结果自动写入 /root/apriltag_config.txt
#
# 也可在主程序中按按键进入编辑器手动调节。

# ==================== 标定参数 ====================
TAG_SIZE_MM   = 100.0        # 标签实际边长 (mm)
KNOWN_DIST_MM = 500.0        # 标签到相机实际距离 (mm)
SAMPLES       = 3            # 采集样本数（按键次数）
RES_W         = 320          # 采集分辨率
RES_H         = 320
# =================================================

from maix import camera, display, image, app, key, time, uart
import math
import apriltag_config as cfg

_FAMILY_MAP = {
    "TAG36H11": image.ApriltagFamilies.TAG36H11,
    "TAG36H10": image.ApriltagFamilies.TAG36H10,
    "TAG25H9":  image.ApriltagFamilies.TAG25H9,
    "TAG25H7":  image.ApriltagFamilies.TAG25H7,
    "TAG16H5":  image.ApriltagFamilies.TAG16H5,
}


def main():
    cfg.load()   # 读取已保存的标签族

    tag_family = _FAMILY_MAP.get(cfg.tag_family, image.ApriltagFamilies.TAG36H11)
    real_diag = TAG_SIZE_MM * math.sqrt(2)

    print("=" * 50)
    print("  AprilTag 焦距标定")
    print("=" * 50)
    print(f"  Family    : {cfg.tag_family}")
    print(f"  Tag size  : {TAG_SIZE_MM} mm")
    print(f"  Known dist: {KNOWN_DIST_MM} mm")
    print(f"  Samples   : {SAMPLES} (按键采集)")
    print("=" * 50)

    cam = camera.Camera(RES_W, RES_H)
    disp = display.Display()

    focal_samples = []
    sample_count = 0

    _kp = [False]
    def _on_key(k, s):
        if s == key.State.KEY_PRESSED:
            _kp[0] = True
    key.Key(_on_key)

    try:
        while not app.need_exit():
            img = cam.read()
            if not img:
                time.sleep_ms(50)
                continue

            tags = img.find_apriltags(families=tag_family)
            best_w = best_h = best_focal = 0

            for t in tags:
                w, h = t.w(), t.h()
                cx, cy = t.cx(), t.cy()
                if w < 10 or h < 10:
                    continue

                diag_px = math.sqrt(w * w + h * h)
                focal = (KNOWN_DIST_MM * diag_px) / real_diag if diag_px > 0 else 0

                if w * h > best_w * best_h:
                    best_w, best_h = w, h
                    best_focal = focal

                # 绘制标签
                try:
                    corners = t.corners()
                    for i in range(4):
                        x1, y1 = int(corners[i][0]), int(corners[i][1])
                        x2, y2 = int(corners[(i+1) % 4][0]), int(corners[(i+1) % 4][1])
                        img.draw_line(x1, y1, x2, y2, image.COLOR_GREEN, 2)
                except:
                    pass
                img.draw_rect(cx - w // 2, cy - h // 2, w, h, image.COLOR_GREEN, 1)
                img.draw_string(cx + 10, cy, f"ID:{t.id()}", image.COLOR_GREEN, 1.0)

            # 按键采集
            if _kp[0]:
                _kp[0] = False
                if best_focal > 0:
                    focal_samples.append(best_focal)
                    sample_count += 1
                    print(f"[Sample {sample_count}/{SAMPLES}] "
                          f"w={best_w} h={best_h} → focal={best_focal:.1f} px")
                else:
                    print(f"[Sample] No tag — skipped")

            # 屏幕信息
            if best_focal > 0:
                img.draw_string(10, 10, f"Focal: {best_focal:.1f}px", image.COLOR_YELLOW, 1.8)
            else:
                img.draw_string(10, 10, "Aim camera at AprilTag", image.COLOR_RED, 1.5)

            img.draw_string(10, 50,
                            f"Samples: {sample_count}/{SAMPLES}  "
                            f"Press KEY to capture",
                            image.COLOR_WHITE, 1.2)
            img.draw_string(10, 75,
                            f"Config: tag={TAG_SIZE_MM}mm  dist={KNOWN_DIST_MM}mm",
                            image.COLOR_WHITE, 0.9)

            disp.show(img)

            if sample_count >= SAMPLES:
                break

            time.sleep_ms(100)

        # ---- 结果 ----
        print()
        if focal_samples:
            avg = sum(focal_samples) / len(focal_samples)
            if len(focal_samples) > 1:
                variance = sum((s - avg) ** 2 for s in focal_samples) / len(focal_samples)
                std = math.sqrt(variance)
            else:
                std = 0.0

            print("=" * 55)
            for i, v in enumerate(focal_samples):
                print(f"  Sample {i+1}: {v:.1f} px")
            print(f"  Average   : {avg:.1f} px")
            if std > 0:
                print(f"  StdDev    : {std:.1f} px")
            print("=" * 55)

            # 自动写入 config
            cfg.focal_length = avg
            cfg.save()
            print(f"\n  ✅ Focal length saved: {avg:.1f} px")
            print(f"  Config: /root/apriltag_config.txt")
        else:
            print("  ❌ No valid samples collected.")
            print("  Make sure tag is visible and values are correct.")

    except Exception as e:
        print(f"Error: {e}")

    finally:
        cam.close()
        print("Camera closed.")


if __name__ == "__main__":
    main()
