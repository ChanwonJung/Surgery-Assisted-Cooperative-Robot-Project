import time
import sys
import rclpy
import DR_init
from robot_control.robot_control import RobotController

from ament_index_python.packages import get_package_share_directory
from robot_control.onrobot import RG

package_path = get_package_share_directory("pick_and_place_voice")

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60
# BUCKET_POS = [445.5, -242.6, 174.4, 156.4, 180.0, -112.5]
GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"
DETECT_WOUND_DEPTH = 70
WAIT_DETECT_WOUND = [51.4, 54.84, 13, 0, 112.11, 51.27]

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# rclpy.init()
# dsr_node = rclpy.create_node("robot_control_node", namespace=ROBOT_ID)
# DR_init.__dsr__node = dsr_node

try:
    from DSR_ROBOT2 import (
        release_compliance_ctrl,
        check_force_condition,
        task_compliance_ctrl,
        movej,
        movel,
        DR_AXIS_Y,
        mwait
    )
except ImportError as e:
    print(f"Error importing DSR_ROBOT2: {e}")
    sys.exit()

########### Gripper Setup. Do not modify this area ############

gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)

########### Robot Controller ############

class DetectWound(RobotController):
    def __init__(self):
        super().__init__('pick_and_place')
        self.force_threshold = 8

    # 오버라이딩으로 코드 재사용
    def pick_and_place_target(self, target_pos, target_type='wound'):
        """
        target_type: 'wound'
        """        
        if target_type == 'wound':  
            release_compliance_ctrl()
            time.sleep(0.5)          
            # 관찰 위치 계산 (목표 위치에서 z축 방향으로 50mm 위로)
            observation_pos = target_pos.copy()
            observation_pos[2]+= DETECT_WOUND_DEPTH
            
            # 관찰 위치로 이동
            print("관찰 위치로 이동합니다.")
            movel(observation_pos, vel=(VELOCITY//2), acc=(ACC//2))
            mwait()
            time.sleep(0.5)

            # 관찰 위치에서 외력 감지
            self.detect_external_force()

    # 외력 감지 함수
    def detect_external_force(self):
        task_compliance_ctrl(stx=[500, 500, 500, 200, 200, 200])
        time.sleep(0.1)

        while True:
            if check_force_condition(DR_AXIS_Y, max=self.force_threshold):
                print("외력 감지! 초기 위치로 복귀합니다.")
                release_compliance_ctrl()
                self.init_robot()
                break
            time.sleep(0.5)
        release_compliance_ctrl()

def main(args=None):
    if not rclpy.ok():
        rclpy.init(args=args)
    
    node = DetectWound()
    while rclpy.ok():
        node.init_robot()
        # 수술 부위를 관찰하기 위한 위치로 이동
        print("수술 부위를 관찰하기 위한 위치로 이동합니다.")
        movej(WAIT_DETECT_WOUND, vel=(VELOCITY//2), acc=(ACC//2))
        mwait()
        time.sleep(2.0)
        node.robot_control()

    rclpy.shutdown()
    node.destroy_node()


if __name__ == "__main__":
    main()