"""ROS 로그에 실제 한국 시간과 Isaac Sim 시간을 함께 표시한다."""

from datetime import datetime
from zoneinfo import ZoneInfo

from rclpy.node import Node


_KST = ZoneInfo("Asia/Seoul")


class _TimestampedLogger:
    """rclpy Logger 호출 앞에 두 시간축을 자동으로 붙이는 래퍼."""

    def __init__(self, logger, node):
        self._logger = logger
        self._node = node

    def _message(self, message):
        wall_time = datetime.now(_KST).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        sim_sec = self._node.get_clock().now().nanoseconds / 1_000_000_000.0
        return f"[KST {wall_time}][SIM {sim_sec:.3f}s] {message}"

    def debug(self, message, *args, **kwargs):
        return self._logger.debug(self._message(message), *args, **kwargs)

    def info(self, message, *args, **kwargs):
        return self._logger.info(self._message(message), *args, **kwargs)

    def warning(self, message, *args, **kwargs):
        return self._logger.warning(self._message(message), *args, **kwargs)

    def warn(self, message, *args, **kwargs):
        return self.warning(message, *args, **kwargs)

    def error(self, message, *args, **kwargs):
        return self._logger.error(self._message(message), *args, **kwargs)

    def fatal(self, message, *args, **kwargs):
        return self._logger.fatal(self._message(message), *args, **kwargs)

    def __getattr__(self, name):
        return getattr(self._logger, name)


class TimestampedNode(Node):
    """기존 ``self.get_logger()`` 사용법을 유지하는 ROS 2 Node."""

    def get_logger(self):
        logger = getattr(self, "_timestamped_logger", None)
        if logger is None:
            logger = _TimestampedLogger(super().get_logger(), self)
            self._timestamped_logger = logger
        return logger
