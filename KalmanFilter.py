try:
    from ulab import numpy as np
    #from ulab import scipy as spy
except ImportError:
    import numpy as np
    #import scipy as spy


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
        self.Xdash = np.zeros(X0.shape)
        self.P = P0.copy()
        self.Pdash = np.zeros(P0.shape)
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

