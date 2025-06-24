import math

#from adafruit_display_shapes.filled_polygon import FilledPolygon
from vectorio import Polygon
from vectorio import Circle
from vectorio import Rectangle
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


class utils:
    @staticmethod
    def ndarray_to_points(arr: np.ndarray) -> list[tuple[int, int]]:
        int_arr = np.array(arr, dtype=np.int16)
        res_points = list(map(tuple, int_arr))
        return res_points


class AltitudeIndicator:
    WIDTH: int = const(240)
    HEIGHT: int = const(240)
    HALF_WIDTH: int = const(WIDTH >> 1)
    HALF_HEIGHT: int = const(HEIGHT >> 1)

    HALF_PI: float = const(math.pi / 2)
    INV_HALF_PI: float = const(1.0 / HALF_PI)

    BLACK_COLOR_VALUE: int = const(0x000000)
    WHITE_COLOR_VALUE: int = const(0xffffff)
    SKY_COLOR_VALUE: int = const(0x019faf)
    GND_COLOR_VALUE: int = const(0xb04602)
    PALETTE_COLOR_COUNT: int = const(4)

    PLANE_COLOR_INDEX: int = const(1)
    SKY_COLOR_INDEX: int = const(2)
    GND_COLOR_INDEX: int = const(3)

    PLANE_BODY_RADIUS: int = const(5)

    roll: float
    pitch: float

    rot_mat: np.ndarray
    move_vec: np.ndarray
    zero_tran_vec: np.ndarray

    palette = displayio.Palette

    main_group: displayio.Group

    back_group: displayio.Group
    poly_sky: Rectangle
    poly_gnd: Rectangle

    plane_group: displayio.Group
    plane_wings: list[np.ndarray]
    plane_body: np.ndarray
    poly_plane_wings: [list[Polygon]]
    poly_plane_body: Circle

    def __init__(self, group: displayio.Group):
        self.roll = 0.0
        self.pitch = 0.0
        #
        self.palette = displayio.Palette(self.PALETTE_COLOR_COUNT)
        self.palette[0] = self.BLACK_COLOR_VALUE
        self.palette[1] = self.WHITE_COLOR_VALUE
        self.palette[2] = self.SKY_COLOR_VALUE
        self.palette[3] = self.GND_COLOR_VALUE
        self.rot_mat = np.array([[0, 0], [0, 0]])
        self.move_vec = np.array([0, 0])
        self.zero_tran_vec = np.array([self.HALF_WIDTH, self.HALF_HEIGHT])
        #
        self.build_groups()
        #
        if group is not None:
            group.append(self.main_group)

    def build_groups(self):
        # sky & gnd
        self.back_group = displayio.Group()
        self.poly_sky = Rectangle(pixel_shader=self.palette, width=self.WIDTH, height=self.HALF_HEIGHT, x=0, y=0, color_index=self.SKY_COLOR_INDEX)
        self.poly_gnd = Rectangle(pixel_shader=self.palette, width=self.WIDTH, height=self.HALF_HEIGHT, x=0, y=self.HALF_WIDTH, color_index=self.GND_COLOR_INDEX)
        self.back_group.append(self.poly_sky)
        self.back_group.append(self.poly_gnd)

        # plane
        # data
        self.plane_body = np.array([0, 0], dtype=np.int16)
        self.plane_wings = [
            np.array([(-60, 0), (-15, 0), (-15, 15)], dtype=np.int16),
            np.array([(15, 15), (15, 0), (60, 0)], dtype=np.int16)
        ]
        # drawings
        self.plane_group = displayio.Group()
        self.poly_plane_body = None
        self.poly_plane_wings = [
            None,
            None
        ]
        self._update_plane()

        self.main_group = displayio.Group()
        self.main_group.append(self.back_group)
        self.main_group.append(self.plane_group)

    #@utimeit
    def _update_rot_mat(self):
        roll_sin = np.sin(self.roll)
        roll_cos = np.cos(self.roll)
        self.rot_mat = np.array([[roll_cos, roll_sin], [-roll_sin, roll_cos]])

    #@utimeit
    def _update_move_vec(self):
        pitch = -max(min(self.pitch, self.HALF_PI), - self.HALF_PI)
        #self.move_vec[0] = 0.0
        self.move_vec[1] = pitch * self.HALF_HEIGHT * self.INV_HALF_PI
        #print(self.move_vec)

    def get_rotated(self, points: np.ndarray) -> np.ndarray:
        res_rect = np.array([np.dot(self.rot_mat, v) for v in points])
        return res_rect

    #@utimeit
    def _update_plane(self):
        pts = self.plane_body + (self.move_vec + self.zero_tran_vec)
        if self.poly_plane_body is not None:
            self.plane_group.remove(self.poly_plane_body)
        self.poly_plane_body = Circle(pixel_shader=self.palette, radius=self.PLANE_BODY_RADIUS, x=int(pts[0]), y=int(pts[1]), color_index=self.PLANE_COLOR_INDEX)
        self.plane_group.append(self.poly_plane_body)

        for i, w in enumerate(self.plane_wings):
            pts = self.get_rotated(w)
            pts += (self.move_vec + self.zero_tran_vec)
            if self.poly_plane_wings[i] is not None:
                self.plane_group.remove(self.poly_plane_wings[i])
            self.poly_plane_wings[i] = Polygon(pixel_shader=self.palette, x=0, y=0, points=utils.ndarray_to_points(pts), color_index=self.PLANE_COLOR_INDEX)
            self.plane_group.append(self.poly_plane_wings[i])


    def update(self):
        self._update_move_vec()
        self._update_rot_mat()
        self._update_plane()

    def paint(self):
        pass