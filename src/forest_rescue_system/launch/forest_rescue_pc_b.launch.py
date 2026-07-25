#!/usr/bin/env python3

"""PC B에서 YOLO 탐지와 RGB-D 위치 추정 노드를 실행한다."""

from forest_rescue_system.bringup.launch_common import (
    ROLE_PC_B,
    make_launch_description,
)


def generate_launch_description():
    return make_launch_description(ROLE_PC_B)
