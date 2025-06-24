import math

from adafruit_display_shapes.filled_polygon import FilledPolygon
from micropython import const

try:
    from ulab import numpy as np
    #from ulab import scipy as spy
except ImportError:
    import numpy as np
    #import scipy as spy

import displayio
import adafruit_display_shapes as shapes
from utimeit import utimeit


class AltitudeIndicator:
    WIDTH: int = const(240)
    HEIGHT: int = const(240)
    HALF_WIDTH: int = const(WIDTH >> 1)
    HALF_HEIGHT: int = const(HEIGHT >> 1)

    HALF_PI: float = const(math.pi / 2)
    INV_HALF_PI = 1.0 / HALF_PI

    SKY_COLOR: int = 0x019faf
    GND_COLOR: int = 0xb04602
    #SKY_GND_OUTLINE_COLOR: int = 0xffffff
    #SKY_GND_OUTLINE_TICKNESS: int = 2

    roll: float
    pitch: float

    rect_sky: np.ndarray
    rect_gnd: np.ndarray

    rot_mat: np.ndarray
    move_vec: np.ndarray
    zero_tran_vec: np.ndarray

    main_group: displayio.Group
    sky_poly: FilledPolygon
    gnd_poly: FilledPolygon

    def __init__(self, group: displayio.Group):
        self.roll = 0.0
        self.pitch = 0.0
        #[0, 0] -> [self.WIDTH-1, self.HEIGHT-1]
        #self.rect_sky = np.array([[0, self.HEIGHT/2], [0, 0], [self.WIDTH-1, 0], [self.WIDTH-1, self.HEIGHT/2]])
        #self.rect_gnd = np.array([[0, self.HEIGHT-1], [0, self.HEIGHT/2], [self.WIDTH-1, self.HEIGHT/2], [self.WIDTH-1, self.HEIGHT-1]])
        #[-self.WIDTH/2, -self.HEIGHT/2] -> [self.WIDTH/2, self.HEIGHT/2]
        # sky
        LEFT: int = -self.HALF_WIDTH
        TOP: int = -self.HALF_HEIGHT
        RIGHT: int = self.HALF_WIDTH - 1
        BOTTOM: int = 0
        self.rect_sky = np.array([[LEFT, TOP], [RIGHT, TOP], [RIGHT, BOTTOM], [LEFT, BOTTOM]], dtype=np.int16)
        # gnd
        LEFT: int = -self.HALF_WIDTH
        TOP: int = 0
        RIGHT: int = self.HALF_WIDTH - 1
        BOTTOM: int = self.HALF_HEIGHT - 1
        self.rect_gnd = np.array([[LEFT, TOP], [RIGHT, TOP], [RIGHT, BOTTOM], [LEFT, BOTTOM]], dtype=np.int16)
        #
        self.rot_mat = np.array([[0, 0], [0, 0]])
        self.move_vec = np.array([0, 0])
        self.zero_tran_vec = np.array([self.HALF_WIDTH, self.HALF_HEIGHT])
        #
        self.build_main_group()
        #
        if group is not None:
            group.append(self.main_group)

    def build_main_group(self):
        self.main_group = displayio.Group()

        self.sky_poly = FilledPolygon(points=[(0, 0)]*4, outline=None, fill=self.SKY_COLOR, close=True, stroke=0)
        self.gnd_poly = FilledPolygon(points=[(0, 0)]*4, outline=None, fill=self.GND_COLOR, close=True, stroke=0)

        self.main_group.append(self.sky_poly)
        self.main_group.append(self.gnd_poly)

    #@utimeit
    def _update_rot_mat(self):
        roll_sin = np.sin(self.roll)
        roll_cos = np.cos(self.roll)
        self.rot_mat = np.array([[roll_cos, -roll_sin], [roll_sin, roll_cos]])

    #@utimeit
    def _update_move_vec(self):
        pitch = max(min(self.pitch, self.HALF_PI), - self.HALF_PI)
        #self.move_vec[0] = 0.0
        self.move_vec[1] = pitch * self.HALF_HEIGHT * self.INV_HALF_PI
        #print(self.move_vec)

    def get_rotated_rect(self, rect: np.ndarray) -> np.ndarray:
        res_rect = np.array([np.dot(self.rot_mat, v) for v in rect])
        return res_rect

    #@utimeit
    def _update_sky_gnd(self):
        # sky
        # pitch
        rect = self.rect_sky.copy()
        rect[2] = rect[2] + self.move_vec
        rect[3] = rect[3] + self.move_vec
        # roll
        rect = self.get_rotated_rect(rect)
        rect += self.zero_tran_vec
        self.sky_poly.points = list(map(tuple, np.array(rect, dtype=np.int16)))

        # ground
        # pitch
        rect = self.rect_gnd.copy()
        rect[0] = rect[0] + self.move_vec
        rect[1] = rect[1] + self.move_vec
        # roll
        rect = self.get_rotated_rect(rect)
        rect += self.zero_tran_vec
        self.gnd_poly.points = list(map(tuple, np.array(rect, dtype=np.int16)))

    def update(self):
        self._update_move_vec()
        self._update_rot_mat()
        self._update_sky_gnd()

    def paint(self):
        pass