# ==================== 可调参数 ====================
IMAGE_WIDTH    = 320
IMAGE_HEIGHT   = 240
BAUDRATE       = 115200
UART_PORT      = "/dev/ttyS0"
LOOP_SLEEP_MS  = 10
SERIAL_OUTPUT  = True
SERIAL_GAP_MS  = 1
CAM_WARMUP     = 30
# ---- 舵机跟踪 ----
TRACK_DEAD      = 15         # 死区 (px)
TRACK_KP        = 0.5        # 比例增益 (<1 防震荡)
TRACK_MAX_STEP  = 5.0        # 单次最大调整角度 (°)
TRACK_LOST_MAX  = 30         # 连续丢帧数超此值→归位 90°
TRACK_SEARCH_DEG   = 3.0       # 丢失时每帧搜索转角 (°)
TRACK_SEARCH_DELAY = 5         # 连续丢帧 N 帧后才判定出画并开始探索
# =================================================

from maix import camera, display, image, uart, app, key, time
import apriltag_config as cfg
import apriltag_detector
import apriltag_editor
import sg90


class AprilTagRanger:
    """AprilTag 测距主程序"""

    def __init__(self):
        # ---- 加载配置 ----
        cfg.load()

        # ---- 相机 ----
        print(f"[Init] Camera {IMAGE_WIDTH}x{IMAGE_HEIGHT}")
        self.cam = camera.Camera(IMAGE_WIDTH, IMAGE_HEIGHT)

        # ---- 显示器 ----
        self.disp = display.Display()
        print(f"[Init] Display {self.disp.width()}x{self.disp.height()}")

        # ---- AprilTag 检测器 ----
        self.detector = apriltag_detector.AprilTagDetector()
        print(f"[Init] family={cfg.tag_family} size={cfg.tag_size_mm}mm "
              f"focal={cfg.focal_length}px area>{cfg.min_area} k1={cfg.distort_k1}")

        # ---- 串口 ----
        self.serial = None
        if SERIAL_OUTPUT:
            try:
                self.serial = uart.UART(UART_PORT, BAUDRATE)
                print(f"[Init] UART {UART_PORT} @ {BAUDRATE}")
            except Exception as e:
                print(f"[Init] UART failed: {e}")

        # ---- 舵机 ----
        try:
            self.servo = sg90.SG90()
            self.servo.set_angle(90)   # 初始朝向正前方
        except Exception as e:
            print(f"[Init] Servo failed: {e}")
            self.servo = None

        # ---- 按键 ----
        self.key_obj = key.Key(self._on_key)
        self._triggered = False

        # ---- 状态 ----
        self.last_results = []
        self.frame_count = 0
        self.mode = "detect"       # "detect" | "editor"
        self._lost_count = 0        # 连续丢失帧数
        self._last_cx = IMAGE_WIDTH // 2   # 最后检测位置
        self._had_detection = False # 是否曾经检测到过 tag
        self._search_done = False   # 本次探索是否已完成

        # ---- 预热 ----
        print(f"[Init] Warming up ({CAM_WARMUP} frames)...")
        for _ in range(CAM_WARMUP):
            self.cam.read()
            time.sleep_ms(20)
        print("[Init] Ready.")

    # ========== 舵机跟踪 ==========
    def _track_servo(self, results):
        """让舵机跟随 tag 位置转动。丢失时根据出画方向旋转探索"""
        if not self.servo:
            return

        # ---- 有检测结果 ----
        if results:
            self._lost_count = 0
            self._search_done = False   # 重置搜索状态
            r = results[0]
            cx = r['cx']
            self._last_cx = cx
            self._had_detection = True  # 标记曾经检测到过 tag
            center = IMAGE_WIDTH / 2.0
            fx = cfg.focal_length

            import math
            offset_deg = math.degrees(math.atan2(center - cx, fx))

            if abs(cx - center) < TRACK_DEAD:
                return

            offset_deg *= TRACK_KP
            offset_deg = max(-TRACK_MAX_STEP, min(TRACK_MAX_STEP, offset_deg))

            target = self.servo.get_angle() + offset_deg
            target = max(0, min(180, target))
            self.servo.set_angle(target)
            return

        # ---- 丢失 tag ----
        # 从未检测到过 → 不探索
        if not self._had_detection:
            return

        self._lost_count += 1

        # 本次探索已完成 → 不动
        if self._search_done:
            return

        # 超时归位，标记搜索完成
        if self._lost_count >= TRACK_LOST_MAX:
            self.servo.set_angle(90)
            self._search_done = True
            print("[Search] timeout → 90° done")
            return

        # 连续丢帧数不足 → 不判定出画，等待
        if self._lost_count < TRACK_SEARCH_DELAY:
            return

        # 出画方向探索（仅一次）
        if self._last_cx < IMAGE_WIDTH / 2:
            new_angle = self.servo.get_angle() + TRACK_SEARCH_DEG
            new_angle = min(180, new_angle)
            self.servo.set_angle(new_angle)
            print(f"[Search] left →{new_angle:.0f}°")
        else:
            new_angle = self.servo.get_angle() - TRACK_SEARCH_DEG
            new_angle = max(0, new_angle)
            self.servo.set_angle(new_angle)
            print(f"[Search] right →{new_angle:.0f}°")

    # ========== 按键 ==========
    def _on_key(self, k, state):
        if state == key.State.KEY_PRESSED:
            self._triggered = True
            print("[Key] Pressed")

    # ========== 帧生成 ==========
    # $31,04,XXX,YYY,F,AAA,BBB,CCC#
    # XXX = 距离 mm (低3位)
    # YYY = 距离 mm 高位 (千位, 0~9)  → 最大 9999mm = 9.999m
    # F   = 5 (AprilTag)
    # AAA = 标签 ID
    # BBB = 倾斜角度×10 (绝对值, deci-degree, 0~999)
    # CCC = 倾斜方向 (0=正/右倾, 1=负/左倾)
    @staticmethod
    def build_frame(distance_mm, rotation_deg, tag_id):
        d = int(round(distance_mm))
        d_lo = d % 1000
        d_hi = min(d // 1000, 9)
        r = abs(rotation_deg)
        r_val = min(int(r * 10), 999)
        r_sign = 0 if rotation_deg >= 0 else 1
        tid = max(0, min(int(tag_id), 999))
        return f"$31,04,{d_lo:03d},{d_hi:03d},5,{tid:03d},{r_val:03d},{r_sign:03d}#\n"

    # ========== 串口发送 ==========
    def _serial_send(self, s: str):
        if not self.serial:
            return
        for b in s.encode():
            self.serial.write(bytes([b]))
            time.sleep_ms(SERIAL_GAP_MS)

    # ========== 输出 ==========
    def _output(self, results):
        import math
        to_send = results[:1] if cfg.output_mode == "nearest" else results

        for r in to_send:
            # 根据 tag 在画面中的横坐标反算偏航角
            # 针孔模型: angle = atan((cx - center) / focal_length)
            cx_center = IMAGE_WIDTH / 2.0
            fx = cfg.focal_length
            angle_deg = math.degrees(math.atan2(r['cx'] - cx_center, fx))

            data = self.build_frame(r['distance_mm'], angle_deg, r['id'])
            self._serial_send(data)
            print(data, end="")

        if not to_send:
            print("No tag")

    # ========== 检测模式一帧 ==========
    def _run_detect(self):
        img = self.cam.read()
        if not img:
            return

        self.frame_count += 1

        # 检测（find_apriltags 内部自动转灰度）+ 绘制
        results = self.detector.detect(img)
        self.detector.draw(img, results)

        self._output(results)
        self.last_results = results
        self.disp.show(img)

        # 舵机跟踪 tag 位置
        self._track_servo(results)

    # ========== 主循环 ==========
    def run(self):
        while not app.need_exit():
            # 按键：切换 detect / editor
            if self._triggered:
                self._triggered = False
                if self.mode == "detect":
                    self.mode = "editor"
                    print("[Main] Switching to editor...")
                else:
                    self.mode = "detect"
                    print("[Main] Switching to detect...")

            if self.mode == "editor":
                editor = apriltag_editor.Editor(self.cam, self.disp)
                # 传入按键回调：第二次按键→退出编辑器
                editor.run(key_cb=lambda: self._triggered)
                self._triggered = False
                self.mode = "detect"
                print("[Main] Back from editor")
                # 编辑器可能改了参数，重新打印
                print(f"[Main] Current: family={cfg.tag_family} "
                      f"size={cfg.tag_size_mm}mm focal={cfg.focal_length}px")
            else:
                self._run_detect()
                time.sleep_ms(LOOP_SLEEP_MS)

        self.cam.close()
        print("[Exit] Done.")


if __name__ == "__main__":
    AprilTagRanger().run()
