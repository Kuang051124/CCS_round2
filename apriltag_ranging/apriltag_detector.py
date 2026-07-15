"""
AprilTag 检测 + 测距 — 直接在输入图像上检测（不做额外预处理）
"""
from maix import image
import math
import apriltag_config as cfg

ALLOWED_IDS = {0, 1, 2}  # 只识别这些 ID 的 tag

_FAMILY_MAP = {
    "TAG36H11": image.ApriltagFamilies.TAG36H11,
    "TAG36H10": image.ApriltagFamilies.TAG36H10,
    "TAG25H9":  image.ApriltagFamilies.TAG25H9,
    "TAG25H7":  image.ApriltagFamilies.TAG25H7,
    "TAG16H5":  image.ApriltagFamilies.TAG16H5,
}

DEBUG_PRINT = False


class AprilTagDetector:

    def __init__(self):
        pass

    @property
    def focal_length(self): return cfg.focal_length
    @property
    def tag_size_mm(self):  return cfg.tag_size_mm
    @property
    def tag_family(self):
        return _FAMILY_MAP.get(cfg.tag_family, image.ApriltagFamilies.TAG36H11)
    @property
    def min_area(self): return cfg.min_area

    def calc_distance(self, w_px, h_px, cx=0, cy=0):
        """计算距离，可选畸变校正"""
        diag_px = math.sqrt(w_px * w_px + h_px * h_px)
        if diag_px <= 0:
            return 0.0
        # 径向畸变校正：桶形畸变使边缘物体变小，需放大补偿
        if cfg.distort_k1 != 0:
            half = cfg.img_size / 2.0
            nx = (cx - half) / half
            ny = (cy - half) / half
            r2 = nx * nx + ny * ny
            scale = 1.0 + cfg.distort_k1 * r2
            diag_px *= scale
        return (self.focal_length * self.tag_size_mm * math.sqrt(2)) / diag_px

    @staticmethod
    def distance_to_str(mm):
        cm = mm / 10.0
        i = int(cm)
        d = int(round((cm - i) * 10))
        if d >= 10: i += 1; d = 0
        return i, d

    def detect(self, img):
        """直接对输入图像做 AprilTag 检测（调用方决定是否预处理）"""
        raw_tags = []
        try:
            raw_tags = img.find_apriltags(families=self.tag_family)
        except Exception as e:
            if DEBUG_PRINT:
                print(f"[AprilTag] detect error: {e}")

        if DEBUG_PRINT and raw_tags:
            print(f"[AprilTag] found: {len(raw_tags)} tags")

        results = []
        for t in raw_tags:
            # 置信度（decision_margin），低置信度=可能误检
            try:
                conf = t.decision_margin()
                if conf < cfg.min_conf:
                    continue   # 置信度不够，丢弃
            except:
                pass

            w, h = t.w(), t.h()
            area = t.area() if hasattr(t, 'area') else w * h
            if area < self.min_area:
                continue
            cx, cy = t.cx(), t.cy()
            if t.id() not in ALLOWED_IDS:
                continue
            try:
                cr = t.corners()
                corners = [(int(c[0]), int(c[1])) for c in cr]
            except:
                corners = []
            try:
                rotation = t.rotation()
            except:
                rotation = 0.0

            dist_mm = self.calc_distance(w, h, cx, cy)
            cm_i, cm_d = self.distance_to_str(dist_mm)

            results.append({
                'id': t.id(), 'family': t.family(),
                'cx': cx, 'cy': cy, 'w': w, 'h': h,
                'area': area, 'corners': corners, 'rotation': rotation,
                'distance_mm': dist_mm, 'cm_int': cm_i, 'cm_dec': cm_d,
                'decision_margin': conf,
            })

        results.sort(key=lambda r: r['distance_mm'])
        return results

    def draw(self, img, results):
        if not results:
            img.draw_string(10, 10, "AprilTag: --", image.COLOR_RED, scale=2.0)
            return

        for r in results:
            cx, cy = r['cx'], r['cy']
            w, h = r['w'], r['h']
            corners = r['corners']

            dist_cm = r['cm_int'] + r['cm_dec'] / 10.0
            if dist_cm < 30:
                color = image.COLOR_GREEN
            elif dist_cm < 80:
                color = image.Color.from_rgb(255, 255, 0)
            else:
                color = image.COLOR_RED

            if len(corners) == 4:
                for i in range(4):
                    x1, y1 = corners[i]
                    x2, y2 = corners[(i + 1) % 4]
                    img.draw_line(x1, y1, x2, y2, color=color, thickness=2)

            img.draw_rect(cx - w // 2, cy - h // 2, w, h, color, thickness=1)
            c = 8
            img.draw_line(cx - c, cy, cx + c, cy, color, 2)
            img.draw_line(cx, cy - c, cx, cy + c, color, 2)
            lbl = f"ID:{r['id']} {r['cm_int']}.{r['cm_dec']}cm"
            img.draw_string(max(0, cx - w // 2), max(0, cy - h // 2 - 22),
                            lbl, color, scale=1.2)

        best = results[0]
        msg = f"Tags:{len(results)} Best:ID{best['id']} {best['cm_int']}.{best['cm_dec']}cm"
        img.draw_string(10, 10, msg, image.COLOR_GREEN, scale=1.5)

    @staticmethod
    def calibrate(known_dist_mm, tag_size_mm, w_px, h_px):
        diag_px = math.sqrt(w_px * w_px + h_px * h_px)
        real_diag = tag_size_mm * math.sqrt(2)
        return (known_dist_mm * diag_px) / real_diag
