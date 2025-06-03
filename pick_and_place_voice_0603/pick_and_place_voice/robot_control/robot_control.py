import os
import time
import sys
from scipy.spatial.transform import Rotation
import numpy as np
import rclpy
from rclpy.node import Node
import DR_init
import json
from object_detection.yolo_surgical import YoloSurgicalModel

from od_msg.srv import SrvDepthPosition
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory
from robot_control.onrobot import RG

package_path = get_package_share_directory("pick_and_place_voice")

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60
GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"
DEPTH_OFFSET = -5.0
MIN_DEPTH = 2.0

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

rclpy.init()
dsr_node = rclpy.create_node("robot_control_node", namespace=ROBOT_ID)
DR_init.__dsr__node = dsr_node

try:
    from DSR_ROBOT2 import (
    posj,
    movej,
    movel,
    DR_FC_MOD_REL,
    DR_BASE,
    mwait,
    get_current_posx,
    movesj,
)
except ImportError as e:
    print(f"Error importing DSR_ROBOT2: {e}")
    sys.exit()

with open(os.path.join(package_path, "resource", "surgical_tools.json"), "r") as f:
    SURGICAL_TOOLS = list(json.load(f).values())  # 클래스 이름 리스트로 추출

########### Gripper Setup. Do not modify this area ############

gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)

########### Robot Controller ############

class RobotController(Node):
    def __init__(self, node_name = 'robot_controller'):
        super().__init__(node_name)
        self.init_robot()


        self.get_position_client = self.create_client(
            SrvDepthPosition, "/get_3d_position"
        )
        while not self.get_position_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for get_depth_position service...")
        self.get_position_request = SrvDepthPosition.Request()

        self.get_keyword_client = self.create_client(Trigger, "/get_keyword")
        while not self.get_keyword_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for get_keyword service...")
        self.get_keyword_request = Trigger.Request()

        self.last_target = None
        self.target_position_map = {}
        

    def get_robot_pose_matrix(self, x, y, z, rx, ry, rz):
        R = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        return T

    def transform_to_base(self, camera_coords, gripper2cam_path, robot_pos):
        """
        Converts 3D coordinates from the camera coordinate system
        to the robot's base coordinate system.
        """
        gripper2cam = np.load(gripper2cam_path)
        coord = np.append(np.array(camera_coords), 1)  # Homogeneous coordinate

        x, y, z, rx, ry, rz = robot_pos
        base2gripper = self.get_robot_pose_matrix(x, y, z, rx, ry, rz)

        # 좌표 변환 (그리퍼 → 베이스)
        base2cam = base2gripper @ gripper2cam
        td_coord = np.dot(base2cam, coord)

        return td_coord[:3]

    def robot_control(self):
        target_list = []
        self.get_logger().info("call get_keyword service")
        self.get_logger().info("say 'Hello Rokey' and speak what you want to pick up")
        get_keyword_future = self.get_keyword_client.call_async(self.get_keyword_request)
        rclpy.spin_until_future_complete(self, get_keyword_future)
        
        if get_keyword_future.result().success:
            get_keyword_result = get_keyword_future.result()
            target_list = get_keyword_result.message.split()

            for target in target_list:
                self.last_target = target
                target_pos = self.get_target_pos(target)         
                if target_pos is None:
                    self.get_logger().warn("No target position")
                else:
                    self.target_position_map[target] = target_pos
                    self.get_logger().info(f"target position: {target_pos}")
                    try:
                        success = self.pick_and_place_target(target_pos)
                        return success
                    except Exception as e:
                        self.get_logger().error(f"Pick and place aborted: {e}")
                        self.init_robot()
                        return False

        else:
            self.get_logger().warn(f"{get_keyword_result.message}")
            return

    def get_target_pos(self, target):
        self.get_position_request.target = target
        self.get_logger().info("call depth position service with object_detection node")
        get_position_future = self.get_position_client.call_async(
            self.get_position_request
        )
        rclpy.spin_until_future_complete(self, get_position_future)

        if get_position_future.result():
            result = get_position_future.result().depth_position.tolist()
            self.get_logger().info(f"Received depth position: {result}")
            if sum(result) == 0:
                print("No target position")
                return None

            gripper2cam_path = os.path.join(
                package_path, "resource", "T_gripper2camera.npy"
            )
            robot_posx = get_current_posx()[0]
            td_coord = self.transform_to_base(result, gripper2cam_path, robot_posx)

            if td_coord[2] and sum(td_coord) != 0:
                td_coord[2] += DEPTH_OFFSET  # DEPTH_OFFSET
                td_coord[2] = max(td_coord[2], MIN_DEPTH)  # MIN_DEPTH: float = 2.0

            target_pos = list(td_coord[:3]) + robot_posx[3:]
            return target_pos
        return None
    
    def is_surgical_tool(self, tool_name: str):
        # 대소문자 구분 없이 비교하려면 양쪽을 소문자 변환해서 비교
        return tool_name.lower() in (tool.lower() for tool in SURGICAL_TOOLS)

    def init_robot(self):
        JReady = [0, 0, 90, 0, 90, 0]
        movej(JReady, vel=VELOCITY, acc=ACC)
        mwait()

    def pick_and_place_target(self, target_pos, target_type='surgical', hand_offset=30.0):
        """
        target_type: 'surgical' or 'hand'
        hand_offset: 손 위에서 얼마나 더 내려갈지(mm)
        """

        if target_type == 'surgical':
            self.last_target_pos = target_pos
            gripper.move_gripper()
            # 1. 목표 위치로 이동
            print("의료 기기가 인식되었습니다. 해당 위치로 기기를 잡으러 이동합니다.")
            movel(target_pos, vel=VELOCITY, acc=ACC)
            mwait()            
            time.sleep(0.5)
         
            # 2. 집은 후 초기좌표로 복귀
            print("의료 기기를 집습니다.")
            gripper.close_gripper()
            while gripper.get_status()[0]:
                time.sleep(0.5)
            mwait()

            time.sleep(0.5)
            self.init_robot()
            time.sleep(0.5)
            
            if self.get_position_request.target == "Hemostat":
                print("봉합 시작")
                suture_start = [51.4, 54.84, 13, 0, 112.11, 51.27]
                suture1 = posj(54.87, 43.42, 59.55, 13.92, 47.14, -81.79)
                suture2 = posj(59.37, 38.61, 67.85, 20.07, 56.91, -202.82)
                suture3 = posj(49.68, 38.34, 69.58, 16.85, 42.95, -62.4)
                suture4 = posj(56.05, 32.95, 78.34, 25.35, 52.11, -214.97)
                suture5 = posj(46.5, 35.39, 76.41, 25.45, 38.8, -69.68)
                suture6 = posj(54.34, 32.25, 78.05, 23.61, 56.35, -212.16)
                suture7 = posj(48.26, 30.31, 74.95, 6.97, 52.14, -126.49)
                suture8 = posj(51.54, 3, 87.15, 13.72, 84.02, -196.53)

                suture_list = [suture1,suture2,suture3,suture4,suture5,suture6,suture7,suture8]
                print("봉합을 시작합니다.")
                time.sleep(1.0)
                mwait()
                movej(suture_start, vel = (VELOCITY//2), acc=(ACC//2))
                mwait()
                time.sleep(1.0)
                movesj(suture_list, vel=(VELOCITY//2), acc=(ACC//2))
                mwait()
                time.sleep(2.0)
                self.init_robot()
                time.sleep(2.0)

            # 3. 수술 위치로 이동
            print("수술 위치로 기기를 이동합니다.")
            destination_pos = [15.66, 4.77, 102.91, 41.97, 66.44, 0]
            movej(destination_pos, vel=(VELOCITY-20), acc=(ACC-20))
            mwait()

            # 4. 손 detection 서비스 재호출
            print("손 좌표를 인식합니다.")
            hand_pos = self.get_hand_position()
            mwait()
            if hand_pos is not None:
                print(f"손 위치로 이동합니다: {hand_pos}")
                self.pick_and_place_target(hand_pos, target_type='hand', hand_offset=hand_offset)
                return True
            else:
                print("손을 인식하지 못했습니다.")
                if self.last_target_pos is not None:
                    print("마지막 위치로 돌아가서 물체를 내려놓습니다.")
                    self.init_robot()
                    movel(self.last_target_pos, vel=VELOCITY, acc=ACC)
                    mwait()
                    time.sleep(0.5)
                    gripper.open_gripper()
                    while gripper.get_status()[0]:
                        time.sleep(0.5)
                    mwait()
                    return False
                else:
                    print("마지막 위치 정보가 없어 로봇을 초기 위치로 복귀합니다.")
                    return False

        elif target_type == 'hand':
            # 1. 손 좌표 위로 이동
            print("손 위치로 이동합니다.")
            movel(target_pos, vel=VELOCITY, acc=ACC)
            mwait()

            # 2. 손 위에서 z축으로 더 내려감
            lower_pos = target_pos.copy()
            lower_pos[2] -= hand_offset  # z축 방향(아래)으로 hand_offset만큼 이동
            movel(lower_pos, vel=VELOCITY, acc=ACC)
            mwait()
            time.sleep(0.5)

            # 3. 그리퍼 열기(놓기)
            gripper.move_gripper()
            while gripper.get_status()[0]:
                time.sleep(0.5)
            mwait()
            return True

    def check_checkpoint(self):
        # 정리해야 할 의료 기기 있는지 확인
        self.init_robot()
        print("[DEBUG] check_checkpoint() 진입")
        print("정리해야 할 물체가 있는지 확인합니다.")
        time.sleep(1.0)
        check_point_pos = [656, -31, 436, 0.8, 172, 0.43]
        movel(check_point_pos, vel=VELOCITY, acc=ACC)
        mwait()
        print("[INFO] checkpoint 도착. YOLO 인식을 기다리는 중...")
        time.sleep(3.0)
        
        # 이전 타겟 정보 초기화
        self.target_position_map.clear()
    
        for tool_name in SURGICAL_TOOLS:
            tool_name_lower = tool_name.lower().strip()
            print(f"[CHECK] '{tool_name}' 검사중 ...")

            # 2. 해당 물체가 체크포인트에 있는지 다시 탐지
            check_target_pos = self.get_target_pos(tool_name_lower)
            time.sleep(1.0)

            if check_target_pos is not None:
                print(f"[CHECK] checkpoint에서 '{tool_name}' 감지됨 → 원래 위치로 되돌리기")
                
                # 원래 위치 정보가 없으므로 기본 위치 설정
                original_pos = self.get_default_storage_position(tool_name)

                # 이동 및 집기
                gripper.move_gripper()
                movel(check_target_pos, vel=VELOCITY, acc=ACC)
                mwait()
                time.sleep(0.5)

                gripper.close_gripper()
                while gripper.get_status()[0]:
                    time.sleep(0.5)
                mwait()
                time.sleep(0.5)

                # 원래 위치로 되돌리기
                self.init_robot()
                time.sleep(0.5)
                movel(original_pos, vel=VELOCITY, acc=ACC)
                mwait()
                gripper.move_gripper()
                while gripper.get_status()[0]:
                    time.sleep(0.5)
                mwait()

                print(f"[COMPLETE] '{tool_name}' 수거완료 ")
                break
        else:
            print("검사 포인트에서 물체를 발견하지 못했습니다.")
            
        self.init_robot()
    
    def get_default_storage_position(self, tool_name):
        """"각 도구별 기본 보관 위치를 반환"""
        # 도구별 보관 좌표
        tool_storage_map = {
            'scalpel': [464, 65, 285, 4, 180, 13],
            'hemostat': [376, 65, 285, 4, 180, 13],
            'forceps': [273, 65, 285, 4, 180, 13],
            'mayo_metz': [175, 65, 285, 4, 180, 13]
        }
        return tool_storage_map.get(tool_name.lower())

    def get_hand_position(self):
        # 손 detection 서비스 호출
        self.get_position_request.target = 'hand' 
        self.get_logger().info("손 위치 인식 서비스 호출")
        time.sleep(2.0)
        get_position_future = self.get_position_client.call_async(self.get_position_request)
        rclpy.spin_until_future_complete(self, get_position_future)

        if get_position_future.result():
            result = get_position_future.result().depth_position.tolist()
            self.get_logger().info(f"손 위치 인식 결과: {result}")
            if sum(result) == 0:
                print("손 위치를 인식하지 못했습니다.")
                return None

            gripper2cam_path = os.path.join(package_path, "resource", "T_gripper2camera.npy")
            robot_posx = get_current_posx()[0]
            td_coord = self.transform_to_base(result, gripper2cam_path, robot_posx)
            if td_coord[2] and sum(td_coord) != 0:
                td_coord[2] += DEPTH_OFFSET
                td_coord[2] = max(td_coord[2], MIN_DEPTH)
            hand_pos = list(td_coord[:3]) + robot_posx[3:]
            self.get_logger().info(f"[HAND] 손 위치 변환 결과 (base frame): {hand_pos}")
            return hand_pos
        return None

def main(args=None):
    node = RobotController()
    try:
        while rclpy.ok():
            node.init_robot()
            gripper.move_gripper()
            success = node.robot_control()
            if success:
                node.get_logger().info("Successfully picked and passed tool to hand.")
                node.check_checkpoint()
            else:
                print("[INFO] 수술 도구 전달 실패. 다시 명령을 기다립니다.")
    except KeyboardInterrupt:
        print("수술 보조를 종료합니다.")
    
    finally:
        rclpy.shutdown()
        node.destroy_node()

if __name__ == "__main__":
    main()