import board
import busio
import fourwire
import terminalio
import displayio
import adafruit_gc9a01a as gc9a01
from adafruit_display_text import bitmap_label
import qmi8658c
import asyncio
import math
import time

try:
    from ulab import numpy as np
    #from ulab import scipy as spy
except ImportError:
    import numpy as np
    #import scipy as spy

from KalmanFilter import KalmanFilter
from AltitudeIndicator import AltitudeIndicator

DT = 0.01

#lcd_spi: busio.SPI = None
#display: gc9a01.GC9A01A = None


class Vec3:
    x: float
    y: float
    z: float

    def __init__(self, x:float=0.0, y:float=0.0, z:float=0.0):
        self.x = x
        self.y = y
        self.z = z

    def __str__(self):
        return f"({self.x},{self.y},{self.z})"


class Orientation:
    roll: float # продольно
    pitch: float # поперечно
    accel_roll: float
    accel_pitch: float
    gyro_roll: float
    gyro_pitch: float
    accel: Vec3
    gyro: Vec3
    raw_accel: Vec3
    raw_gyro: Vec3
    temp: float
    dt: float

    def __init__(self):
        self.roll = 0.0
        self.pitch = 0.0
        self.accel_roll = 0.0
        self.accel_pitch = 0.0
        self.gyro_roll = 0.0
        self.gyro_pitch = 0.0
        self.accel = Vec3()
        self.gyro = Vec3()
        self.raw_accel = Vec3()
        self.raw_gyro = Vec3()
        self.temp = 0.0
        self.dt = 0.0

class Calibration:
    accel: Vec3
    gyro: Vec3

    def __init__(self):
        self.accel = Vec3(-0.100216722972972, 1.35917060472973, 0.0153553790169492)
        #self.accel_gain = Vec3(-1.0, 0.0, 0.0)
        self.gyro = Vec3(0.001169915805, -0.056912726, 0.0150643665)
        #self.gyro_gain = Vec3(-1.0, 0.0, 0.0)


orient: Orientation = Orientation()
calibr: Calibration = Calibration()

def init_lcd():
    #global lcd_spi
    #global display
    # Raspberry Pi Pico pinout, one possibility, at "southwest" of board
    tft_clk = board.GP10  # must be a SPI CLK
    tft_mosi = board.GP11  # must be a SPI TX
    tft_rst = board.GP12
    tft_dc = board.GP8
    tft_cs = board.GP9  # optional, can be "None"
    tft_bl = board.GP25  # optional, can be "None"
    lcd_spi = busio.SPI(clock=tft_clk, MOSI=tft_mosi)
    display_bus = fourwire.FourWire(lcd_spi, command=tft_dc, chip_select=tft_cs, reset=tft_rst, baudrate=100000000)
    display = gc9a01.GC9A01A(display_bus, width=240, height=240, backlight_pin=tft_bl, auto_refresh=False)
    return display


def init_imu():
    imu_sda = board.IMU_SDA
    imu_scl = board.IMU_SCL
    i2c = busio.I2C(scl=imu_scl, sda=imu_sda)
    imu = qmi8658c.QMI8658C(i2c)
    return imu

async def imu_handle(imu):
    global orient
    global calibr
    global DT

    X0 = np.array([[0],
                   [0]])
    P0 = np.array([[1, 0],
                   [0, 1]])
    F = np.array([[1, DT],
                  [0, 1]])
    B = np.array([[0],
                  [DT]])
    Q = np.array([[0.001, 0],
                  [0, 0.01]])
    H = np.array([1, 0])
    R = np.array([0.01])

    kf_roll = KalmanFilter(X0, P0, F, B, Q, H, R)
    kf_pitch = KalmanFilter(X0, P0, F, B, Q, H, R)

    acc_x: float = 0.0
    acc_y: float = 0.0
    acc_z: float = 0.0
    acc_roll: float = 0.0
    acc_pitch: float = 0.0

    gyro_x: float = 0.0
    gyro_y: float = 0.0
    gyro_z: float = 0.0
    gyro_droll: float = 0.0
    gyro_dpitch: float = 0.0
    gyro_roll: float = 0.0
    gyro_pitch: float = 0.0

    temp: float = 0.0

    roll: float = 0.0
    pitch: float = 0.0

    last_time: float = time.monotonic()
    cur_time: float = last_time
    dt: float = 0.0

    while True:
        #print("Updating imu...")
        # current time.
        cur_time = time.monotonic()
        #
        acc_x, acc_y, acc_z = imu.acceleration
        gyro_x, gyro_y, gyro_z = imu.gyro
        temperature = imu.temperature
        #
        orient.raw_accel.x = acc_x
        orient.raw_accel.y = acc_y
        orient.raw_accel.z = acc_z
        orient.raw_gyro.x = gyro_x
        orient.raw_gyro.y = gyro_y
        orient.raw_gyro.z = gyro_z
        #
        acc_x = acc_x - calibr.accel.x
        acc_y = acc_y - calibr.accel.y
        acc_z = acc_z - calibr.accel.z
        gyro_x = gyro_x - calibr.gyro.x
        gyro_y = gyro_y - calibr.gyro.y
        gyro_z = gyro_z - calibr.gyro.z
        #
        orient.accel.x = acc_x
        orient.accel.y = acc_y
        orient.accel.z = acc_z
        orient.gyro.x = gyro_x
        orient.gyro.y = gyro_y
        orient.gyro.z = gyro_z
        orient.temp = temp
        #
        # accel
        acc_roll = math.atan2(-acc_x, acc_y)
        acc_pitch = math.atan2(-acc_x, -acc_z)
        #
        # gyro
        dt = cur_time - last_time
        gyro_droll = -gyro_z
        gyro_dpitch = -gyro_y
        gyro_roll = orient.roll + gyro_droll * dt
        gyro_pitch = orient.pitch + gyro_dpitch * dt
        # filter
        #alpha = 0.94
        #roll = gyro_roll * alpha + (1.0 - alpha) * acc_roll
        #pitch = gyro_pitch * alpha + (1.0 - alpha) * acc_pitch
        kf_roll.calc(gyro_droll, acc_roll)
        kf_pitch.calc(gyro_dpitch, acc_pitch)
        roll = kf_roll.X[0][0]
        pitch = kf_pitch.X[0][0]
        #
        orient.accel_roll = acc_roll
        orient.accel_pitch = acc_pitch
        orient.gyro_roll = gyro_roll
        orient.gyro_pitch = gyro_pitch
        orient.roll = roll
        orient.pitch = pitch
        orient.dt = dt
        #
        last_time = time.monotonic()
        await asyncio.sleep(DT)

async def lcd_handle(display):
    global orient
    # root group
    main_group = displayio.Group()
    display.root_group = main_group

    alt_ind = AltitudeIndicator(main_group)

    # label
    text_group = displayio.Group(scale=2, x=50, y=120)
    text = "Hello World!"
    text_area = bitmap_label.Label(terminalio.FONT, text=text, color=0xFFFF00)
    text_group.append(text_area)  # Subgroup for text scaling
    main_group.append(text_group)

    half_pi = math.pi / 2

    roll: float = 0.0
    pitch: float = 0.0

    while True:
        roll = orient.roll - half_pi
        pitch = orient.pitch - half_pi

        #print("DT:", orient.dt)
        #print("A", orient.accel, "G", orient.gyro)
        #print("A", orient.raw_accel.x, orient.raw_accel.y, orient.raw_accel.z, "G", orient.raw_gyro.x, orient.raw_gyro.y, orient.raw_gyro.z, sep=", ")
        #print(orient.dt, orient.raw_accel.x, orient.raw_accel.y, orient.raw_accel.z, orient.raw_gyro.x,
        #      orient.raw_gyro.y, orient.raw_gyro.z, sep=", ")

        alt_ind.roll = roll
        alt_ind.pitch = pitch
        alt_ind.update()
        text_area.text = f"{orient.pitch * 180 / math.pi}\n{orient.roll * 180 / math.pi}"
        display.refresh()
        await asyncio.sleep(0.1)

async def run():
    displayio.release_displays()
    display = init_lcd()

    imu = init_imu()

    imu_task = asyncio.create_task(imu_handle(imu))
    lcd_task = asyncio.create_task(lcd_handle(display))

    await asyncio.gather(imu_task, lcd_task)


def main():
    asyncio.run(run())


if __name__ == '__main__':
    main()
