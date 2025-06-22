import numpy as np
import matplotlib.pyplot as plt

IDLE_SAMPLES = 200
DT = 0.01
ALPHA = 0.9
accel_cal = [-0.100216722972972, 1.35917060472973, 0.0153553790169492]
gyro_cal = [0.001169915805, -0.056912726, 0.0150643665]

# def calibrate(data, data_cal):
#     rows = np.size(data, 0)
#     cols = np.size(data, 1)
#
#     res_data = np.zeros(rows, cols)
#
#     for i in range(rows):
#         res_data[i, 0] = data[i, 0] - data_cal[0]
#         res_data[i, 1] = data[i, 1] - data_cal[1]
#         res_data[i, 2] = data[i, 2] - data_cal[2]
#
#     return res_data

import numpy as np


def calibrate(data, data_cal):
    data_size = data.shape
    rows = data_size[0]
    cols = data_size[1]

    res_data = np.zeros((rows, cols))

    for i in range(rows):
        res_data[i, 0] = data[i, 0] - data_cal[0]
        res_data[i, 1] = data[i, 1] - data_cal[1]
        res_data[i, 2] = data[i, 2] - data_cal[2]

    return res_data


def calc_accel_roll_pitch(data):
    data_size = data.shape
    rows = data_size[0]

    res_roll = np.zeros((rows, 1))
    res_pitch = np.zeros((rows, 1))

    for i in range(rows):
        acc_x = data[i, 0]
        acc_y = data[i, 1]
        acc_z = data[i, 2]

        roll = np.arctan2(-acc_x, acc_y)
        pitch = np.arctan2(-acc_x, -acc_z)

        res_roll[i] = roll
        res_pitch[i] = pitch

    return res_roll, res_pitch


def calc_gyro_droll_dpitch(data):
    data_size = data.shape
    rows = data_size[0]

    res_droll = np.zeros((rows, 1))
    res_dpitch = np.zeros((rows, 1))

    for i in range(rows):
        gyro_x = data[i, 0]
        gyro_y = data[i, 1]
        gyro_z = data[i, 2]

        droll = -gyro_z
        dpitch = -gyro_y

        res_droll[i] = droll
        res_dpitch[i] = dpitch

    return res_droll, res_dpitch


def calc_roll_pitch(accel_roll, accel_pitch, gyro_droll, gyro_dpitch, dt, alpha):
    data_size = accel_roll.shape
    rows = data_size[0]

    res_roll = np.zeros((rows, 1))
    res_pitch = np.zeros((rows, 1))

    roll = 0
    pitch = 0

    for i in range(rows):
        acc_roll = accel_roll[i]
        acc_pitch = accel_pitch[i]

        gy_droll = gyro_droll[i]
        gy_dpitch = gyro_dpitch[i]

        gy_roll = roll + gy_droll * dt
        gy_pitch = pitch + gy_dpitch * dt

        roll = gy_roll * alpha + (1.0 - alpha) * acc_roll
        pitch = gy_pitch * alpha + (1.0 - alpha) * acc_pitch

        res_roll[i] = roll
        res_pitch[i] = pitch

    return res_roll, res_pitch


class KalmanFilter:
    def __init__(self, X0, P0, F, B, Q, H, R):
        """
        Инициализация фильтра Калмана

        Параметры:
        X0 - начальное состояние (вектор)
        P0 - начальная ковариационная матрица ошибки оценки
        F - матрица перехода состояния
        B - матрица управления
        Q - ковариационная матрица шума процесса
        H - матрица измерений
        R - ковариационная матрица шума измерений
        """
        self.X = X0.copy()
        self.Xdash = np.zeros_like(X0)
        self.P = P0.copy()
        self.Pdash = np.zeros_like(P0)
        self.F = F.copy()
        self.FT = F.T.copy()
        self.B = B.copy()
        self.Q = Q.copy()
        self.H = H.copy()
        #self.HT = H.T.copy()
        self.HT = H.copy().reshape((-1, 1))
        self.R = R.copy()
        self.I = np.eye(X0.shape[0])
        #self.U = np.array([0, 0])

    def predict(self, u):
        """
        Этап предсказания фильтра Калмана

        Параметры:
        u - вектор управления
        """
        #self.Xdash = np.dot(self.F, self.X) + np.dot(self.B, u)
        self.Xdash = np.dot(self.F, self.X) + self.B * u
        self.Pdash = np.dot(np.dot(self.F, self.P), self.FT) + self.Q

    def update(self, z):
        """
        Этап обновления фильтра Калмана

        Параметры:
        z - вектор измерений
        """
        S = np.dot(np.dot(self.H, self.Pdash), self.HT) + self.R
        #invS = np.linalg.inv(S)
        invS = 1.0 / S

        #K = np.dot(np.dot(self.Pdash, self.HT), invS)
        K = np.dot(self.Pdash, self.HT) * invS

        y = z - np.dot(self.H, self.Xdash)

        #self.X = self.Xdash + np.dot(K, y)
        self.X = self.Xdash + K * y
        #self.P = np.dot((self.I - np.dot(K, self.H)), self.Pdash)
        self.P = np.dot((self.I - K * self.H), self.Pdash)

    def calc(self, u, z):
        """
        Полный цикл фильтра Калмана (предсказание + обновление)

        Параметры:
        u - вектор управления
        z - вектор измерений
        """
        self.predict(u)
        self.update(z)


#

data = np.loadtxt("accel_gyro_data.csv", delimiter=',')

SAMPLES = len(data)
t = np.linspace(0, DT * (SAMPLES - 1), SAMPLES)

accel_data = calibrate(data[:, 0:3], accel_cal)
gyro_data = calibrate(data[:, 3:6], gyro_cal)

accel_roll, accel_pitch = calc_accel_roll_pitch(accel_data)
gyro_droll, gyro_dpitch = calc_gyro_droll_dpitch(gyro_data)

roll, pitch = calc_roll_pitch(accel_roll, accel_pitch, gyro_droll, gyro_dpitch, DT, ALPHA)

X0 = np.array([[0],
               [0]])
P0 = np.array([[1, 0],
               [0, 1]])
F = np.array([[1, DT],
              [0, 1 ]])
B = np.array([[0 ],
              [DT]])
Q = np.array([[0.001, 0   ],
              [0,     0.01]])
H = np.array([1, 0])
R = np.array([0.01])

kf_roll = KalmanFilter(X0, P0, F, B, Q, H, R)
kf_pitch = KalmanFilter(X0, P0, F, B, Q, H, R)

roll_kf = np.zeros(SAMPLES)
pitch_kf = np.zeros(SAMPLES)

for i in range(SAMPLES):
    kf_roll.calc(gyro_droll[i], accel_roll[i])
    roll_kf[i] = kf_roll.X[0]

    kf_pitch.calc(gyro_dpitch[i], accel_pitch[i])
    pitch_kf[i] = kf_pitch.X[0]
#


# Создаем массив x от 1 до SAMPLES с шагом 1
#x = np.arange(1, SAMPLES + 1)
x = t

# Создаем график
plt.figure(figsize=(10, 6))

# Рисуем линии с соответствующими цветами и стилями
plt.plot(x, accel_roll, 'r', label='Accel roll')
plt.plot(x, accel_pitch, 'b', label='Accel pitch')
plt.plot(x, roll_kf, 'm', label='Kalman roll')
plt.plot(x, pitch_kf, 'c', label='Kalman pitch')
plt.plot(x, roll, 'm--', label='AB roll')
plt.plot(x, pitch, 'c--', label='AB pitch')

# Добавляем легенду
plt.legend()

# Устанавливаем пределы осей
plt.xlim(x[200-1], x[400-1])
plt.ylim(-3.5, 3.5)

# Включаем сетку
plt.grid(True, which='both', linestyle='--', alpha=0.6)
plt.minorticks_on()
plt.grid(True, which='minor', linestyle=':', alpha=0.4)

# Показываем график
plt.show()
