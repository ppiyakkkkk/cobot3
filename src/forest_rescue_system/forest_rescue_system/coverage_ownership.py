#!/usr/bin/env python3

"""삼각형별 소유 드론을 관리한다 (먼저 본 드론이 소유권 유지)."""

import numpy as np


class TriangleOwnership:
    def __init__(self, triangle_count):
        self._owner = np.full(triangle_count, -1, dtype=np.int32)

    @property
    def owner_ids(self):
        return self._owner

    def unclaimed_mask(self):
        return self._owner < 0

    def claim(self, triangle_indices, drone_index):
        indices = np.asarray(triangle_indices, dtype=np.int64)
        if indices.size == 0:
            return np.asarray([], dtype=np.int64)
        unclaimed = indices[self._owner[indices] < 0]
        self._owner[unclaimed] = drone_index
        return unclaimed

    def indices_for_drone(self, drone_index):
        return np.where(self._owner == drone_index)[0]
