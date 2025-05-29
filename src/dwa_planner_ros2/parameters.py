import yaml
import math
from rclpy.node import Node

class Parameters:
    def __init__(self, node: Node):
        defaults = {
            'ANGLE_RESOLUTION': 0.087,
            # ... (생략: 이전에 정의된 기본값 모두 포함) ...
            'YAWRATE_SAMPLES': 20
        }
        params_file = node.declare_parameter('config_file', 'config/params.yaml').value
        with open(params_file, 'r') as f:
            file_params = yaml.safe_load(f)
        for key, default in {**defaults, **file_params}.items():
            node.declare_parameter(key, default)
        self._load(node)

    def _load(self, node: Node):
        p = lambda x: node.get_parameter(x).value
        self.angle_resolution = p('ANGLE_RESOLUTION')
        # ... (이하 모든 파라미터 로드) ...
        self.yawrate_samples = p('YAWRATE_SAMPLES')

    def print(self, node: Node):
        log = node.get_logger().info
        log('=== DWA Planner Parameters ===')
        for attr in vars(self):
            log(f'{attr}: {getattr(self, attr)}')
