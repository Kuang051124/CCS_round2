"""
AprilTag 参数编辑器 — 调节 k1 畸变系数
"""
from maix import camera, display, image, touchscreen, app, time, key as _key
import _thread
import apriltag_config as cfg
import apriltag_detector

PANEL_W  = 160
BTN_H    = 38
BTN_GAP  = 6
PAD      = 8

C_BG     = image.Color.from_rgb(20, 20, 25)
C_PANEL  = image.Color.from_rgb(30, 30, 38)
C_BTN    = image.Color.from_rgb(50, 55, 65)
C_GREEN  = image.Color.from_rgb(0, 185, 80)
C_RED    = image.Color.from_rgb(210, 50, 50)
C_WHITE  = image.COLOR_WHITE
C_GRAY   = image.Color.from_rgb(120, 120, 130)
C_YELLOW = image.Color.from_rgb(255, 255, 0)


class Editor:

    def __init__(self, cam, disp):
        self.cam = cam
        self.disp_w = disp.width()
        self.disp_h = disp.height()
        self.cam_w = self.disp_w - PANEL_W
        self.cam_h = self.disp_h
        self.disp = disp
        self.detector = apriltag_detector.AprilTagDetector()

        self._tx = 0; self._ty = 0; self._pressed = False
        try:
            self.ts = touchscreen.TouchScreen()
            _thread.start_new_thread(self._touch_thread, ())
        except:
            self.ts = None

        self._key_pressed = False
        # 按键由外部注入，不自己注册
        self._exit = False
        self._do_cal = False
        self._cal_ok = 0

    def on_key(self):
        """外部注入按键事件"""
        self._key_pressed = True

    def _touch_thread(self):
        while not app.need_exit() and not self._exit:
            try:
                x, y, pressed = self.ts.read()
                self._tx = x; self._ty = y; self._pressed = pressed
            except:
                pass
            time.sleep_ms(40)

    @staticmethod
    def _hit(x, y, bx, by, bw, bh):
        return bx <= x <= bx + bw and by <= y <= by + bh

    def _draw_btn(self, img, x, y, w, h, text, color=C_BTN, tc=C_WHITE, sc=0.55):
        img.draw_rect(x, y, w, h, color, thickness=-1)
        tw = image.string_size(text, scale=sc).width()
        th = image.string_size(text, scale=sc).height()
        img.draw_string(x + (w - tw) // 2, y + (h - th) // 2, text, tc, scale=sc)

    def _draw_panel(self, img):
        px = self.cam_w + PAD
        py = PAD
        pw = PANEL_W - PAD * 2

        img.draw_rect(self.cam_w, 0, PANEL_W, self.disp_h, C_PANEL, thickness=-1)
        img.draw_line(self.cam_w, 0, self.cam_w, self.disp_h, C_WHITE, 1)

        # ---- 当前参数 ----
        img.draw_string(px, py, "Config", C_GREEN, scale=0.65)
        py += 22
        lines = [
            f"Fam: {cfg.tag_family}",
            f"Tag: {cfg.tag_size_mm:.0f}mm",
            f"Focal: {cfg.focal_length:.0f}",
            f"Area>{cfg.min_area}",
            f"k1: {cfg.distort_k1:.3f}",
            f"Out: {cfg.output_mode}",
        ]
        for ln in lines:
            img.draw_string(px, py, ln, C_WHITE, scale=0.45)
            py += 17

        py += 10

        # ---- k1 调节 ----
        img.draw_string(px, py, "Distort k1", C_GREEN, scale=0.6)
        py += 20
        img.draw_string(px, py, f"{cfg.distort_k1:.3f}", C_YELLOW, scale=2.0)
        py += 50

        bw = (pw - PAD) // 2
        self._draw_btn(img, px, py, bw, 40, "-.01", C_BTN, C_WHITE, 0.6)
        self._draw_btn(img, px + bw + PAD, py, bw, 40, "+.01", C_GREEN, C_WHITE, 0.6)
        py += 44
        self._draw_btn(img, px, py, pw, 28, "k1=0", C_BTN, C_WHITE, 0.5)
        py += 36

        # ---- 一键标定 ----
        cal_lbl = f"Auto Cal ({cfg.cal_dist_mm:.0f}mm)"
        self._draw_btn(img, px, py, pw, BTN_H, cal_lbl, C_GREEN if self._cal_ok <= 0 else image.Color.from_rgb(255,255,0), C_WHITE, 0.55)
        py += BTN_H + BTN_GAP

        # ---- 操作 ----
        self._draw_btn(img, px, py, pw, BTN_H, "Save", C_GREEN)
        py += BTN_H + BTN_GAP
        self._draw_btn(img, px, py, pw, BTN_H, "Load", C_BTN)
        py += BTN_H + BTN_GAP
        self._draw_btn(img, px, py, pw, BTN_H, "Reset", C_BTN)
        py += BTN_H + BTN_GAP
        self._draw_btn(img, px, py, pw, BTN_H, "Exit", C_RED)

    def _handle_touch(self, x, y):
        if not self._pressed:
            return False

        px = self.cam_w + PAD
        # skip past config text + k1 label + value
        py = PAD + 22 + 17*6 + 10 + 20 + 50
        pw = PANEL_W - PAD * 2

        # k1 +/- buttons
        bw = (pw - PAD) // 2
        if self._hit(x, y, px, py, bw, 40):
            cfg.distort_k1 = round(max(0, cfg.distort_k1 - 0.01), 3); return True
        if self._hit(x, y, px + bw + PAD, py, bw, 40):
            cfg.distort_k1 = round(min(1.0, cfg.distort_k1 + 0.01), 3); return True
        py += 44

        # k1=0
        if self._hit(x, y, px, py, pw, 28):
            cfg.distort_k1 = 0.0; return True
        py += 36

        # Auto Cal
        if self._hit(x, y, px, py, pw, BTN_H):
            self._do_cal = True; return True
        py += BTN_H + BTN_GAP

        # Save / Load / Reset / Exit
        if self._hit(x, y, px, py, pw, BTN_H):
            cfg.save(); return True
        py += BTN_H + BTN_GAP
        if self._hit(x, y, px, py, pw, BTN_H):
            cfg.load(); return True
        py += BTN_H + BTN_GAP
        if self._hit(x, y, px, py, pw, BTN_H):
            cfg.reset_defaults(); return True
        py += BTN_H + BTN_GAP
        if self._hit(x, y, px, py, pw, BTN_H):
            self._exit = True; cfg.save(); return True

        return False

    def run(self, key_cb=None):
        print("[Editor] k1 editor (press key to exit)")
        self._exit = False
        self._key_pressed = False

        while not self._exit and not app.need_exit():
            # 检查外部按键
            if key_cb and key_cb():
                self._exit = True; break
            if self._key_pressed:
                self._key_pressed = False
                self._exit = True; break

            img = self.cam.read()
            if not img:
                time.sleep_ms(30); continue

            # 检测（find_apriltags 内部自动转灰度）+ 绘制
            results = self.detector.detect(img)

            # 一键标定
            if self._do_cal:
                self._do_cal = False
                if results:
                    r = results[0]
                    diag = (r['w']**2 + r['h']**2) ** 0.5
                    if diag > 0:
                        new_f = (cfg.cal_dist_mm * diag) / (cfg.tag_size_mm * 1.414)
                        cfg.focal_length = round(new_f, 1)
                        cfg.save()
                        self._cal_ok = 15   # 闪烁 15 帧
                        print(f"[AutoCal] focal={cfg.focal_length:.1f}")
                if not self._cal_ok:
                    print("[AutoCal] No tag found!")

            if self._cal_ok > 0:
                self._cal_ok -= 1

            self.detector.draw(img, results)

            # canvas
            canvas = self._make_canvas(img)
            self._draw_panel(canvas)
            self.disp.show(canvas)
            self._handle_touch(self._tx, self._ty)
            time.sleep_ms(60)

        cfg.save()
        print("[Editor] Done.")

    def _make_canvas(self, cam_img):
        canvas = image.Image(self.disp_w, self.disp_h, image.Format.FMT_RGB888)
        canvas.draw_rect(0, 0, self.disp_w, self.disp_h, C_BG, thickness=-1)
        scale = min(self.cam_w / cam_img.width(), self.cam_h / cam_img.height())
        dw = int(cam_img.width() * scale)
        dh = int(cam_img.height() * scale)
        if dw > 0 and dh > 0:
            resized = cam_img.resize(dw, dh)
            canvas.draw_image((self.cam_w - dw) // 2, (self.cam_h - dh) // 2, resized)
        return canvas
