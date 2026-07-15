"""
SG90 舵机 — MaixCAM PWM 驱动
接线: 橙→A19, 红→5V, 棕→GND
"""
from maix import pwm, time, pinmap, err, sys

# ==================== 参数 ====================
PWM_FREQ   = 50       # 50Hz, 周期 20ms
MIN_DUTY   = 2.5      # 0°: 0.5ms → 2.5%
MAX_DUTY   = 12.2     # 180°: 2.5ms → 12.2%
# =============================================


class SG90:
    """SG90 舵机 0°~180°"""

    def __init__(self, pin="A19"):
        # 根据引脚确定 PWM 通道
        pwm_cfg = {
            "A19": ("A19", 7),
            "A18": ("A18", 6),
        }
        pin_name, pwm_id = pwm_cfg.get(pin, (pin, 7))

        err.check_raise(
            pinmap.set_pin_function(pin_name, f"PWM{pwm_id}"),
            f"set pinmap {pin_name}→PWM{pwm_id} failed"
        )

        self.pwm = pwm.PWM(pwm_id, freq=PWM_FREQ, duty=MIN_DUTY, enable=True)
        self._angle = 90
        print(f"[SG90] {pin_name}→PWM{pwm_id} ready")

    def set_angle(self, degrees):
        """0°~180°"""
        degrees = max(0, min(180, degrees))
        duty = (MAX_DUTY - MIN_DUTY) * degrees / 180.0 + MIN_DUTY
        self.pwm.duty(duty)
        self._angle = degrees
        print(f"[SG90] set_angle({degrees}°) duty={duty:.1f}%")

    def get_angle(self):
        return self._angle

    def stop(self):
        self.pwm.duty(0)

    @staticmethod
    def angle_to_duty(degrees):
        return (MAX_DUTY - MIN_DUTY) * max(0, min(180, degrees)) / 180.0 + MIN_DUTY
