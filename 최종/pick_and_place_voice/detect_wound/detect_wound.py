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
GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"

DETECT_WOUND_DEPTH = 40 # 상처 부위 추적 높이
WAIT_DETECT_WOUND = [51.4, 54.84, 13, 0, 112.11, 51.27] # 상처 부위 추적 시작 위치
JReady = [0, 0, 90, 0, 90, 0] # 로봇 초기 위치
YANKAUER_GRIP_POS = [425, -227, 277, 42.85, 179.97, 42.9] # 석션 잡으러 가는 위치
YANKAUER_POS = [425, -227, 382.3, 42.85, 179.97, 42.9] # 석션 잡기 위한 위치
# DReady = [367.21, 7.23, 423.43, 31.15, 179.95, 31.01] # movec로 초기 좌표로 이동할 위치
# D_MID = [23.36, 329.4, 413.08, 43.31, -179.48, 42.36] # movec로 초기 좌표로 이동할 때 경유 위치

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
        mwait,
        movec,
        move_periodic,
        DR_BASE
    )
except ImportError as e:
    print(f"[ERROR] Error importing DSR_ROBOT2: {e}")
    sys.exit()

########### Gripper Setup. Do not modify this area ############

gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)

########### Robot Controller ############

class DetectWound(RobotController):
    def __init__(self):
        super().__init__('pick_and_place')
        self.force_threshold = 8

    # 수술 부위를 탐지 위치로 이동
    def detect_wound_pos(self):
        print("[INFO] 수술 부위 탐지를 위해 이동합니다.")
        movej(WAIT_DETECT_WOUND, vel=(VELOCITY//2), acc=(ACC//2))
        mwait()
        time.sleep(2.0)

    # 수술 부위 관찰 위치로 이동
    def move_to_observation_pos(self, target_pos):
        print("[INFO] 수술 부위 관찰을 위해 이동합니다.")
        observation_pos = target_pos.copy()
        observation_pos[2] += DETECT_WOUND_DEPTH
        movel(observation_pos, vel=(VELOCITY//2), acc=(ACC//2))
        mwait()
        time.sleep(0.5)
        return observation_pos

    # 외력 감지 함수
    def detect_external_force(self):
        print("[WARN] 순응 제어를 실시합니다. 외력이 감지될 시 초기 위치로 복귀합니다. ")
        release_compliance_ctrl()
        task_compliance_ctrl(stx=[480, 480, 480, 180, 180, 180])
        time.sleep(0.1)

        while True:
            if check_force_condition(DR_AXIS_Y, max=self.force_threshold):
                print("[WARN] 외력이 감지되었습니다! ")
                release_compliance_ctrl()
                self.init_robot()
                break
            time.sleep(0.5)
        release_compliance_ctrl()

    # 오버라이딩으로 코드 재사용
    def pick_and_place_target(self, target_pos, target_type='wound'):
        """
        target_type: 'wound' 또는 'Yankauer'
        """        
        if target_type == 'Yankauer':
            time.sleep(0.5)
            # 수술 부위 감지 후 석션 작업
            if target_pos is not None:
                # 관찰 위치로 이동
                observation_pos = target_pos.copy()
                DETECT_WOUND_DEPTH_1 = 115
                observation_pos = target_pos.copy()
                observation_pos[2] += DETECT_WOUND_DEPTH_1
                time.sleep(1.0)
                print("[INFO] 수술 부위 관찰을 위해 이동합니다.")
                movel(observation_pos, vel=(VELOCITY//2), acc=(ACC//2))
                mwait()

                time.sleep(1.0)
                print("[CHECK] 석션 작업을 진행합니다.")
                move_periodic([3,0,10,0,0,0], period=5.0, atime=0.5, repeat=5, ref=DR_BASE)  # 5초 주기로 왕복
                time.sleep(5.0)  # 5초 동안 석션 작업 수행
                
                self.detect_external_force()    # 외력 감지 시작
                mwait()
                time.sleep(2.0)
                self.init_robot()
                movel(YANKAUER_POS, vel=VELOCITY, acc=ACC)
                mwait()
                time.sleep(2.0)
                movel(YANKAUER_GRIP_POS, vel=VELOCITY, acc=ACC)
                mwait()
                gripper.move_gripper(width_val=700)
                time.sleep(2.0)
                print("[INFO] 석션을 원위치 하였습니다.")
                movel(YANKAUER_POS, vel=VELOCITY, acc=ACC)
                mwait()
                time.sleep(1.0)
                self.init_robot()
                
            
        elif target_type == 'wound':
            time.sleep(0.5)          
            
            # 관찰 위치로 이동
            print("[CHECK] 진행중인 수술 관찰 위치로 이동합니다.")
            self.move_to_observation_pos(target_pos)

            # 관찰 위치에서 외력 감지
            self.detect_external_force()

    # 오버라이딩으로 용도 변경
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

                if target == "Yankauer":
                    try:
                        # 1. 석션을 집을 위치로 이동
                        self.get_logger().info(f"석션 위치: {YANKAUER_POS}")
                        self.init_robot()
                        print("[INFO] 석션을 집으러 이동합니다.")
                        movec(JReady, YANKAUER_POS, vel=VELOCITY, acc=ACC)
                        mwait()
                        
                        # 그리퍼를 70mm로 열기 (RG2의 최대 너비)
                        gripper.move_gripper(width_val=700)
                        time.sleep(2.0)
                        movel(YANKAUER_GRIP_POS, vel=VELOCITY, acc=ACC)
                        mwait()

                        # 그리퍼를 3mm로 닫기
                        gripper.move_gripper(width_val = 30, force_val = 500)
                        time.sleep(2.0)
                        print("[INFO] 석션을 성공적으로 잡았습니다.")
                        movel(YANKAUER_POS, vel=VELOCITY, acc=ACC)
                        mwait()

                        # 2. 수술 부위 탐지 위치로 이동
                        print("[INFO] 수술 부위 탐지 위치로 이동합니다.")
                        self.init_robot()
                        mwait()
                        movej(WAIT_DETECT_WOUND, vel=(VELOCITY-20), acc=(ACC-20))
                        # movec(JReady, WAIT_DETECT_WOUND, vel=VELOCITY, acc=ACC)
                        mwait()
                        time.sleep(3.0)
                        
                        # 3. wound 찾아서 관찰 및 석션 작업
                        wound_pos = self.get_target_pos("wound")
                        mwait()
                        time.sleep(1.0)
                        print("[CHECK] 수술 부위 좌표는", wound_pos)
                        if wound_pos is not None:
                            self.pick_and_place_target(wound_pos, target_type='Yankauer')
                        else:
                            print("[WARN] 석션이 제대로 잡혀있지 않습니다.")
                        return
                    except Exception as e:
                        self.get_logger().error(f"동작이 거절되었습니다.: {e}")
                        self.init_robot()
                        return
                    
                elif target == "wound":  # wound인 경우에만 모델 인식 수행
                    target_pos = self.get_target_pos(target)
                    if target_pos is None:
                        self.get_logger().warn("수술 부위가 인식되지 않았았습니다.")
                    else:
                        self.get_logger().info(f"현재 수술 부위의 위치는: {target_pos}")
                        try:
                            self.pick_and_place_target(target_pos)
                            return
                        except Exception as e:
                            self.get_logger().error(f"동작이 거절되었습니다. {e}")
                            self.init_robot()
                            return
                        
                elif target == "종료":
                    print("[INFO] 수술 보조 작업을 종료합니다.")
                    self.init_robot()
                    return True
                else:
                    self.get_logger().warn(f"{target}이 인식되어 수술 부위가 인식되지 않고 있습니다.")
                    return
        else:
            self.get_logger().warn(f"{get_keyword_result.message}")
            return

def main(args=None):
    if not rclpy.ok():
        rclpy.init(args=args)
    
    node = DetectWound()
    while rclpy.ok():
        node.init_robot()
        node.detect_wound_pos()
        if node.robot_control():
            break

    rclpy.shutdown()
    node.destroy_node()


if __name__ == "__main__":
    main()