"""
钢球坐标、速度测量与 STM32F103C8T6 UART 数据发送

平台：01Studio CanMV K230 + MIPI(ST7701)显示屏 + STM32F103C8T6 最小系统板

STM32F103C8T6 默认 USART1 引脚（不使用重映射）：
    PA9  = USART1_TX
    PA10 = USART1_RX

实际接线（3.3 V TTL，必须共地）：
    K230 Pin 8  / IO3 / TX1  --> STM32 PA10 / USART1_RX
    K230 Pin 10 / IO4 / RX1  <--  STM32 PA9  / USART1_TX
    K230 Pin 9  / GND        ---  STM32 GND

注意：C8T6 也可重映射 USART1 至 PB6/PB7，但本程序和接线均使用默认 PA9/PA10，
STM32 端不要开启 USART1 重映射。

UART 帧：115200, 8N1，固定 10 字节，约 30 Hz：
    AA 55 | VER | SEQ | X_L X_H | V_L V_H | VALID | CRC8

    VER    : 0x01
    X      : int16 little-endian, round(x_cm * 100)，单位 0.01 cm
    V      : int16 little-endian, round(v_cm_s * 100)，单位 0.01 cm/s
    VALID  : 1=钢球有效，0=丢球/无效
    CRC8   : CRC-8/ATM(poly=0x07)，覆盖 VER 至 VALID 共 7 字节

STM32 端收到且 CRC 正确后：
    x_cm = int16_le(X_L, X_H) / 100.0
    v_cm_s = int16_le(V_L, V_H) / 100.0
    valid = VALID
"""

import time
import os
import gc

from machine import UART
from machine import FPIOA
from media.sensor import *
from media.display import *
from media.media import *


# ============================================================================
# 1. 相机、ROI、识别和标定参数
# ============================================================================

CAM_WIDTH = 320
CAM_HEIGHT = 240
LCD_WIDTH = 800
LCD_HEIGHT = 480
CAMERA_ID = 2

ROI_X = 20
ROI_Y = 110
ROI_W = 280
ROI_H = 20
ROI = (ROI_X, ROI_Y, ROI_W, ROI_H)

BACKGROUND_WAIT_MS = 5000
BACKGROUND_STABILIZE_FRAMES = 5

DIFF_THRESHOLD = 40
BALL_MIN_PIXELS = 30
BALL_MAX_PIXELS = 2000
BALL_MIN_WIDTH = 6
BALL_MAX_WIDTH = 22
BALL_MIN_HEIGHT = 6
BALL_MAX_HEIGHT = 20

# 三点标定：记录球心位于图像左侧 5 cm、O 点、图像右侧 5 cm 时的像素 x。
LEFT_5CM_PIXEL = 104.0
ROD_CENTER_PIXEL = 160.0
RIGHT_5CM_PIXEL = 216.0

# 题目中的 +5 cm 位于图像右边时保持 1.0；位于左边则改为 -1.0。
X_SIGN = 1.0
CENTERLINE_Y_TOLERANCE_PX = 12


# ============================================================================
# 2. 位置/速度估计和运行性能参数
# ============================================================================

ALPHA = 0.60
BETA = 0.10
MAX_BALL_SPEED_CM_S = 55.0
MIN_ALLOWED_JUMP_PX = 14.0
LOST_RESET_FRAMES = 5

DISPLAY_INTERVAL_MS = 67
GC_PERIOD_FRAMES = 30
PRINT_DEBUG = False


# ============================================================================
# 3. K230 UART1 与 STM32F103C8T6 默认 USART1 参数
# ============================================================================

# K230 UART1：排针 Pin 8(IO3/TX1)、Pin 10(IO4/RX1)。
UART_TX_IO = 3
UART_RX_IO = 4
UART_BAUDRATE = 115200
UART_SEND_INTERVAL_MS = 33

FRAME_HEAD_0 = 0xAA
FRAME_HEAD_1 = 0x55
FRAME_VERSION = 0x01
FRAME_LENGTH = 10


# ============================================================================
# 4. 工具、坐标换算和图像叠加
# ============================================================================

def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


def ms_diff(now_ms, old_ms):
    return time.ticks_diff(now_ms, old_ms)


def raw_pixel_to_cm(pixel_x):
    """图像坐标：左侧为负、右侧为正；未包含题目正方向 X_SIGN。"""
    if pixel_x >= ROD_CENTER_PIXEL:
        pixels_per_cm = (RIGHT_5CM_PIXEL - ROD_CENTER_PIXEL) / 5.0
    else:
        pixels_per_cm = (ROD_CENTER_PIXEL - LEFT_5CM_PIXEL) / 5.0

    if pixels_per_cm < 0.1:
        pixels_per_cm = 0.1
    return (pixel_x - ROD_CENTER_PIXEL) / pixels_per_cm


def pixel_to_cm(pixel_x):
    return X_SIGN * raw_pixel_to_cm(pixel_x)


def cm_to_pixel(x_cm):
    """将物理坐标转回像素，用于参考线和位置预测。"""
    raw_cm = X_SIGN * x_cm
    if raw_cm >= 0.0:
        pixels_per_cm = (RIGHT_5CM_PIXEL - ROD_CENTER_PIXEL) / 5.0
    else:
        pixels_per_cm = (ROD_CENTER_PIXEL - LEFT_5CM_PIXEL) / 5.0

    if pixels_per_cm < 0.1:
        pixels_per_cm = 0.1
    return ROD_CENTER_PIXEL + raw_cm * pixels_per_cm


def estimated_pixels_per_cm():
    left_scale = abs(ROD_CENTER_PIXEL - LEFT_5CM_PIXEL) / 5.0
    right_scale = abs(RIGHT_5CM_PIXEL - ROD_CENTER_PIXEL) / 5.0
    return max(left_scale, right_scale, 1.0)


def show_on_lcd(img):
    Display.show_image(
        img,
        x=round((LCD_WIDTH - CAM_WIDTH) / 2),
        y=round((LCD_HEIGHT - CAM_HEIGHT) / 2)
    )


def draw_text(img, x, y, text):
    img.draw_string_advanced(x, y, 18, text, color=255)


def draw_roi_and_reference(img):
    img.draw_rectangle(ROI_X, ROI_Y, ROI_W, ROI_H, color=255, thickness=2)
    for reference_cm in (-5.0, 0.0, 5.0):
        line_x = round(cm_to_pixel(reference_cm))
        if ROI_X <= line_x < ROI_X + ROI_W:
            # 若需要细化中线，将下一行改为 line_thickness = 1。
            line_thickness = 1
            img.draw_line(
                line_x,
                ROI_Y,
                line_x,
                ROI_Y + ROI_H - 1,
                color=255,
                thickness=line_thickness
            )


# ============================================================================
# 5. 钢球候选筛选与 alpha-beta 位置/速度估计
# ============================================================================

def choose_ball_blob(blobs, predicted_full_x, max_jump_px):
    """按尺寸、中心线距离、形状和预测位置连续性筛选钢球。"""
    best_blob = None
    best_score = None
    best_quality = 0
    roi_center_y = ROI_H / 2.0

    for blob in blobs:
        width = blob.w()
        height = blob.h()
        pixels = blob.pixels()

        if pixels < BALL_MIN_PIXELS or pixels > BALL_MAX_PIXELS:
            continue
        if width < BALL_MIN_WIDTH or width > BALL_MAX_WIDTH:
            continue
        if height < BALL_MIN_HEIGHT or height > BALL_MAX_HEIGHT:
            continue

        full_x = ROI_X + blob.cx()
        center_y_error = abs(blob.cy() - roi_center_y)
        if center_y_error > CENTERLINE_Y_TOLERANCE_PX:
            continue

        if predicted_full_x is None:
            x_error = 0.0
        else:
            x_error = abs(full_x - predicted_full_x)
            if x_error > max_jump_px:
                continue

        shape_error = abs(width - height)
        score = pixels * 2.0 - x_error * 14.0 - center_y_error * 5.0 - shape_error * 4.0
        if best_score is None or score > best_score:
            best_score = score
            best_blob = blob
            best_quality = int(clamp(100.0 - x_error * 4.0 - shape_error * 3.0, 0.0, 100.0))

    return best_blob, best_quality


class AlphaBetaEstimator:
    """对位置和速度同时滤波，输出单位分别为 cm、cm/s。"""

    def __init__(self):
        self.valid = False
        self.x_cm = 0.0
        self.v_cm_s = 0.0
        self.last_ms = 0

    def reset(self):
        self.valid = False
        self.v_cm_s = 0.0

    def predict(self, now_ms):
        if not self.valid:
            return None, 0.0
        dt_s = clamp(ms_diff(now_ms, self.last_ms) / 1000.0, 0.0, 0.20)
        return self.x_cm + self.v_cm_s * dt_s, dt_s

    def update(self, measured_x_cm, now_ms):
        if not self.valid:
            self.valid = True
            self.x_cm = measured_x_cm
            self.v_cm_s = 0.0
            self.last_ms = now_ms
            return self.x_cm, self.v_cm_s, 0.0

        dt_s = ms_diff(now_ms, self.last_ms) / 1000.0
        self.last_ms = now_ms
        if dt_s <= 0.0 or dt_s > 0.20:
            self.x_cm = measured_x_cm
            self.v_cm_s = 0.0
            return self.x_cm, self.v_cm_s, dt_s

        predicted_x_cm = self.x_cm + self.v_cm_s * dt_s
        residual_cm = measured_x_cm - predicted_x_cm
        self.x_cm = predicted_x_cm + ALPHA * residual_cm
        self.v_cm_s = self.v_cm_s + BETA * residual_cm / dt_s
        return self.x_cm, self.v_cm_s, dt_s


# ============================================================================
# 6. K230 UART1 数据帧编码和发送
# ============================================================================

def crc8_atm(data):
    """CRC-8/ATM：poly=0x07、init=0x00、无反射。"""
    crc = 0
    for byte_value in data:
        crc ^= byte_value
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def int16_to_le_bytes(value):
    value = int(clamp(value, -32768, 32767))
    if value < 0:
        value += 65536
    return value & 0xFF, (value >> 8) & 0xFF


class Stm32F103C8T6UartSender:
    """发送固定 10 字节帧；C8T6 默认 USART1 在 PA9/PA10。"""

    def __init__(self):
        fpioa = FPIOA()
        fpioa.set_function(UART_TX_IO, FPIOA.UART1_TXD)
        fpioa.set_function(UART_RX_IO, FPIOA.UART1_RXD)
        self.uart = UART(
            UART.UART1,
            baudrate=UART_BAUDRATE,
            bits=UART.EIGHTBITS,
            parity=UART.PARITY_NONE,
            stop=UART.STOPBITS_ONE
        )
        self.sequence = 0
        self.sent_count = 0
        self.error_count = 0

    def send_measurement(self, x_cm, v_cm_s, valid):
        if valid:
            x_scaled = int(round(x_cm * 100.0))
            v_scaled = int(round(v_cm_s * 100.0))
        else:
            # STM32 收到 valid=0 时必须忽略 x/v；置零便于示波或串口观察。
            x_scaled = 0
            v_scaled = 0

        x_low, x_high = int16_to_le_bytes(x_scaled)
        v_low, v_high = int16_to_le_bytes(v_scaled)
        payload = [
            FRAME_VERSION,
            self.sequence,
            x_low,
            x_high,
            v_low,
            v_high,
            1 if valid else 0
        ]
        frame = bytes([FRAME_HEAD_0, FRAME_HEAD_1] + payload + [crc8_atm(payload)])

        try:
            write_result = self.uart.write(frame)
            if write_result is not None and write_result != FRAME_LENGTH:
                self.error_count += 1
                return False
        except Exception:
            self.error_count += 1
            return False

        self.sequence = (self.sequence + 1) & 0xFF
        self.sent_count += 1
        return True

    def deinit(self):
        try:
            self.uart.deinit()
        except Exception:
            pass


# ============================================================================
# 7. K230 相机、MIPI 和背景初始化
# ============================================================================

sensor = Sensor(id=CAMERA_ID, width=1920, height=1080, fps=90)
sensor.reset()
sensor.set_framesize(width=CAM_WIDTH, height=CAM_HEIGHT)
sensor.set_pixformat(Sensor.GRAYSCALE)

Display.init(
    Display.ST7701,
    width=LCD_WIDTH,
    height=LCD_HEIGHT,
    to_ide=True
)
MediaManager.init()
sensor.run()


def capture_background():
    print("Remove ball from ROI for background capture")
    wait_start_ms = time.ticks_ms()
    while ms_diff(time.ticks_ms(), wait_start_ms) < BACKGROUND_WAIT_MS:
        img = sensor.snapshot()
        draw_roi_and_reference(img)
        draw_text(img, 5, 5, "REMOVE BALL")
        draw_text(img, 5, 28, "WAIT %ds" % (BACKGROUND_WAIT_MS // 1000))
        show_on_lcd(img)
        time.sleep_ms(50)

    background_full = None
    for _ in range(BACKGROUND_STABILIZE_FRAMES):
        background_full = sensor.snapshot()
        time.sleep_ms(30)
    return background_full, background_full.copy(roi=ROI)


background_full, background = capture_background()
ready_img = background_full.copy()
draw_roi_and_reference(ready_img)
draw_text(ready_img, 5, 5, "BACKGROUND OK")
draw_text(ready_img, 5, 28, "C8T6 UART READY")
show_on_lcd(ready_img)
time.sleep_ms(1000)


# ============================================================================
# 8. 主循环：识别 -> x/v 估计 -> UART1 发送 -> MIPI 显示
# ============================================================================

clock = time.clock()
estimator = AlphaBetaEstimator()
uart_sender = Stm32F103C8T6UartSender()

lost_frames = 0
frame_count = 0
last_display_ms = None
last_uart_send_ms = None
last_uart_ok = False

try:
    while True:
        clock.tick()
        now_ms = time.ticks_ms()
        frame_count += 1

        img = sensor.snapshot()
        current_roi = img.copy(roi=ROI)
        diff = current_roi.copy()
        diff.difference(background)
        diff.binary([(DIFF_THRESHOLD, 255)])

        blobs = diff.find_blobs(
            [(128, 255)],
            pixels_threshold=BALL_MIN_PIXELS,
            area_threshold=BALL_MIN_PIXELS
        )

        predicted_x_cm, prediction_dt_s = estimator.predict(now_ms)
        if predicted_x_cm is None:
            predicted_full_x = None
            max_jump_px = MIN_ALLOWED_JUMP_PX
        else:
            predicted_full_x = cm_to_pixel(predicted_x_cm)
            candidate_dt_s = clamp(prediction_dt_s, 0.02, 0.10)
            max_jump_px = max(
                MIN_ALLOWED_JUMP_PX,
                MAX_BALL_SPEED_CM_S * estimated_pixels_per_cm() * candidate_dt_s
            )

        ball_blob, quality = choose_ball_blob(blobs, predicted_full_x, max_jump_px)
        measurement_valid = ball_blob is not None
        x_cm = 0.0
        v_cm_s = 0.0
        raw_x_cm = 0.0
        ball_y = 0

        if measurement_valid:
            lost_frames = 0
            ball_x = ROI_X + ball_blob.cx()
            ball_y = ROI_Y + ball_blob.cy()
            raw_x_cm = pixel_to_cm(ball_x)
            x_cm, v_cm_s, _ = estimator.update(raw_x_cm, now_ms)

            img.draw_rectangle(
                ROI_X + ball_blob.x(),
                ROI_Y + ball_blob.y(),
                ball_blob.w(),
                ball_blob.h(),
                color=255,
                thickness=2
            )
            img.draw_circle(round(cm_to_pixel(x_cm)), ball_y, 5, color=255, thickness=2)
        else:
            lost_frames += 1
            if lost_frames >= LOST_RESET_FRAMES:
                estimator.reset()

        # 始终发送：检测失败时发送 valid=0，STM32 不能继续使用旧位置/速度。
        if last_uart_send_ms is None or ms_diff(now_ms, last_uart_send_ms) >= UART_SEND_INTERVAL_MS:
            last_uart_ok = uart_sender.send_measurement(x_cm, v_cm_s, measurement_valid)
            last_uart_send_ms = now_ms

        draw_roi_and_reference(img)
        fps_now = clock.fps()
        if measurement_valid:
            draw_text(img, 5, 5, "BALL OK Q:%d" % quality)
            draw_text(img, 5, 28, "X:%+.2fcm" % x_cm)
            draw_text(img, 5, 51, "V:%+.2fcm/s" % v_cm_s)
        else:
            draw_text(img, 5, 5, "BALL LOST")
            draw_text(img, 5, 28, "UART VALID=0")

        uart_text = "OK" if last_uart_ok else "ERR"
        draw_text(img, 5, 74, "U:%s S:%d E:%d" % (
            uart_text,
            uart_sender.sequence,
            uart_sender.error_count
        ))
        draw_text(img, 170, 5, "FPS:%.1f" % fps_now)

        if PRINT_DEBUG:
            print("valid=%d raw=%+.3f x=%+.3f v=%+.3f q=%d uart=%s seq=%d fps=%.1f" % (
                measurement_valid,
                raw_x_cm,
                x_cm,
                v_cm_s,
                quality,
                uart_text,
                uart_sender.sequence,
                fps_now
            ))

        if last_display_ms is None or ms_diff(now_ms, last_display_ms) >= DISPLAY_INTERVAL_MS:
            show_on_lcd(img)
            last_display_ms = now_ms

        if frame_count % GC_PERIOD_FRAMES == 0:
            gc.collect()

finally:
    uart_sender.deinit()
    sensor.stop()
    Display.deinit()
    MediaManager.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
