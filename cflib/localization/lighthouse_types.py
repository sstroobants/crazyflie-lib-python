# -*- coding: utf-8 -*-
#
# ,---------,       ____  _ __
# |  ,-^-,  |      / __ )(_) /_______________ _____  ___
# | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
# | / ,--'  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
# Copyright (C) 2022 Bitcraze AB
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, in version 3.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.
from typing import NamedTuple

import numpy as np
import numpy.typing as npt
from scipy.spatial.transform import Rotation

from cflib.localization.lighthouse_bs_vector import LighthouseBsVectors


class Pose:
    """ Holds the full pose (position and orientation) of an object.
    Contains functionality to convert between various formats."""

    _NO_ROTATION_MTX = np.identity(3)
    _NO_ROTATION_VCT = np.array((0.0, 0.0, 0.0))
    _ORIGIN = np.array((0.0, 0.0, 0.0))

    def __init__(self, R_matrix: npt.ArrayLike = _NO_ROTATION_MTX, t_vec: npt.ArrayLike = _ORIGIN) -> None:
        # Rotation as a matix
        self._R_matrix = np.array(R_matrix)

        # Translation vector
        self._t_vec = np.array(t_vec)

    @classmethod
    def from_rot_vec(cls, R_vec: npt.ArrayLike = _NO_ROTATION_VCT, t_vec: npt.ArrayLike = _ORIGIN) -> 'Pose':
        """
        Create a Pose from a rotation vector and translation vector
        """
        return Pose(Rotation.from_rotvec(R_vec).as_matrix(), t_vec)

    @property
    def rot_matrix(self) -> npt.NDArray:
        """
        Get the rotation matrix of the pose
        """
        return self._R_matrix

    @property
    def rot_vec(self) -> npt.NDArray:
        """
        Get the rotation vector of the pose
        """
        return Rotation.from_matrix(self._R_matrix).as_rotvec()

    @property
    def translation(self) -> npt.NDArray:
        """
        Get the translation vector of the pose
        """
        return self._t_vec

    @property
    def matrix_vec(self) -> tuple[npt.NDArray, npt.NDArray]:
        """
        Get the pose as a rotation matrix and translation vector
        """
        return self._R_matrix, self._t_vec

    def rotate_translate(self, point: npt.ArrayLike) -> npt.NDArray:
        """
        Rotate and translate a point, that is transform from local
        to global reference frame
        """
        return np.dot(self.rot_matrix, point) + self.translation

    def inv_rotate_translate(self, point: npt.ArrayLike) -> npt.NDArray:
        """
        Inverse rotate and translate a point, that is transform from global
        to local reference frame
        """
        return np.dot(np.transpose(self.rot_matrix), point - self.translation)


class LhMeasurement(NamedTuple):
    """Represents a measurement from one base station."""
    timestamp: float
    base_station_id: int
    angles: LighthouseBsVectors


class LhCfPoseSample:
    """ Represents a sample of a Crazyflie pose in space, it contains
    various data related to the pose such as:
    - lighthouse angles from one or more base stations
    - initial estimate of the pose
    - refined estimate of the pose
    - estimated errors
    """

    def __init__(self, timestamp: float = 0.0, angles_calibrated: dict[int, LighthouseBsVectors] = None) -> None:
        self.timestamp: float = timestamp

        # Angles measured by the Crazyflie and compensated using calibration data
        # Stored in a dictionary using base station id as the key
        self.angles_calibrated: dict[int, LighthouseBsVectors] = angles_calibrated
        if self.angles_calibrated is None:
            self.angles_calibrated = {}

        # Initial estimates of bs poses for this sample, in the CF reference frame
        self.initial_est_bs_poses: dict[int, Pose] = {}

        # The initial estimate of the CF pose for this sample, in the global ref frame
        self.inital_est_pose: Pose = None

        # The refined estimate of the CF pose for this sample, in the global ref frame
        self.estimated_pose: Pose = None

        # The aprroximate errors in final solution
        self.estimated_errors: dict[int, float] = {}


class LhDeck4SensorPositions:
    """ Positions of the sensors on the Lighthouse 4 deck """
    # Sensor distances on the lighthouse deck
    _sensor_distance_width = 0.015
    _sensor_distance_length = 0.03

    # Sensor positions in the Crazyflie reference frame
    positions = np.float32([
        (-_sensor_distance_length / 2, _sensor_distance_width / 2, 0.0),
        (-_sensor_distance_length / 2, -_sensor_distance_width / 2, 0.0),
        (_sensor_distance_length / 2, _sensor_distance_width / 2, 0.0),
        (_sensor_distance_length / 2, -_sensor_distance_width / 2, 0.0)])
