#!/usr/bin/env python3

"""PC A에서 제어·임무·경로·시각화 노드를 실행한다."""

from forest_rescue_system.bringup.launch_common import (
    ROLE_PC_A,
    make_launch_description,
)


def generate_launch_description():
    return make_launch_description(ROLE_PC_A)
