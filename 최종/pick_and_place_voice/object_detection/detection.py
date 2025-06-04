import numpy as np
import rclpy
from rclpy.node import Node
from typing import Any, Callable, Optional, Tuple

from ament_index_python.packages import get_package_share_directory
from od_msg.srv import SrvDepthPosition
from object_detection.realsense import ImgNode
from object_detection.yolo_surgical import YoloSurgicalModel
from object_detection.yolo_hands import YoloHandModel
from object_detection.yolo_wound import YoloWoundModel


PACKAGE_NAME = 'pick_and_place_voice'
PACKAGE_PATH = get_package_share_directory(PACKAGE_NAME)


class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        self.img_node = ImgNode()
        self.medical_model = YoloSurgicalModel()
        self.medical_classes = [k.lower().strip() for k in self.medical_model.reversed_class_dict.keys()]

        self.hand_model = YoloHandModel()
        self.hand_classes = [k.lower().strip() for k in self.hand_model.reversed_class_dict.keys()]

        self.wound_model = YoloWoundModel()
        self.wound_classes = [k.lower().strip() for k in self.wound_model.reversed_class_dict.keys()]

        self.intrinsics = self._wait_for_valid_data(
            self.img_node.get_camera_intrinsic, "camera intrinsics"
        )
        self.create_service(
            SrvDepthPosition,
            'get_3d_position',
            self.handle_get_depth
        )
        self.get_logger().info("ObjectDetectionNode 초기화 완료 했습니다.")

    # def _load_model(self, name):
    #     """모델 이름에 따라 인스턴스를 반환합니다."""
    #     if name.lower() == 'yolo':
    #         return YoloModel()
    #     raise ValueError(f"Unsupported model: {name}")

    def handle_get_depth(self, request, response):
        """클라이언트 요청을 처리해 3D 좌표를 반환합니다."""
        self.get_logger().info(f"Received request: {request}")
        coords = self._compute_position(request.target)
        response.depth_position = [float(x) for x in coords]
        return response

    def _compute_position(self, target):
        """이미지를 처리해 객체의 카메라 좌표를 계산합니다."""
        target = target.lower().strip()
        # hand 모델 우선(충돌 방지)
        if target in self.hand_classes:
            model = self.hand_model
        elif target in self.medical_classes:
            model = self.medical_model
        elif target in self.wound_classes:
            model = self.wound_model
        else:
            self.get_logger().error(f"{target}은 없는 객체입니다. ")
            return 0.0, 0.0, 0.0

        box, score = model.get_best_detection(self.img_node, target)
        

        if box is None or score is None:
            self.get_logger().warn("객체를 찾을 수 없습니다. ")
            return 0.0, 0.0, 0.0
        
        self.get_logger().info(f"Detection: box={box}, score={score}")
        cx, cy = map(int, [(box[0] + box[2]) / 2, (box[1] + box[3]) / 2])
        cz = self._get_depth(cx, cy)
        if cz is None:
            self.get_logger().warn("Depth 값이 범위를 벗어났습니다. ")
            return 0.0, 0.0, 0.0

        return self._pixel_to_camera_coords(cx, cy, cz)

    def _get_depth(self, x, y):
        """픽셀 좌표의 depth 값을 안전하게 읽어옵니다."""
        frame = self._wait_for_valid_data(self.img_node.get_depth_frame, "depth frame")
        try:
            return frame[y, x]
        except IndexError:
            self.get_logger().warn(f"좌표 ({x},{y})가 범위를 벗어났습니다. ")
            return None

    def _wait_for_valid_data(self, getter, description):
        """getter 함수가 유효한 데이터를 반환할 때까지 spin 하며 재시도합니다."""
        data = getter()
        while data is None or (isinstance(data, np.ndarray) and not data.any()):
            rclpy.spin_once(self.img_node)
            self.get_logger().info(f"{description}을 받기 위해 재시도합니다.")
            data = getter()
        return data

    def _pixel_to_camera_coords(self, x, y, z):
        """픽셀 좌표와 intrinsics를 이용해 카메라 좌표계로 변환합니다."""
        fx = self.intrinsics['fx']
        fy = self.intrinsics['fy']
        ppx = self.intrinsics['ppx']
        ppy = self.intrinsics['ppy']
        return (
            (x - ppx) * z / fx,
            (y - ppy) * z / fy,
            z
        )


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
