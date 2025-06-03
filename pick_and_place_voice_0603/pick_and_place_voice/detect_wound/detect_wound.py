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

DETECT_WOUND_DEPTH = 35 # 상처 부위 추적 높이
WAIT_DETECT_WOUND = [51.4, 54.84, 13, 0, 112.11, 51.27] # 상처 부위 추적 시작 위치
JReady = [0, 0, 90, 0, 90, 0] # 로봇 초기 위치
YANKAUER_GRIP_POS = [420.26, -225.84, 282.3, 42.85, 179.97, 42.9] # 석션 잡으러 가는 위치
YANKAUER_POS = [420.26, -225.84, 382.3, 42.85, 179.97, 42.9] # 석션 잡기 위한 위치
DReady = [367.21, 7.23, 423.43, 31.15, 179.95, 31.01] # movec로 초기 좌표로 이동할 위치
D_MID = [23.36, 329.4, 413.08, 43.31, -179.48, 42.36] # movec로 초기 좌표로 이동할 때 경유 위치

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
        move_periodic
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

    # 로봇 초기 위치로 이동하는 함수(movec 사용)
    def init_robot(self):
        # movec를 사용하여 곡선 경로로 이동
        movec(D_MID, DReady, vel=(VELOCITY//2), acc=(ACC//2))
        #gripper.open_gripper()
        mwait()

    # 수술 부위를 관찰하기 위한 위치로 이동
    def detect_wound_pos(self):
        print("수술 부위를 관찰하기 위한 위치로 이동합니다.")
        movej(WAIT_DETECT_WOUND, vel=(VELOCITY//2), acc=(ACC//2))
        mwait()
        time.sleep(2.0)

    # 명령 대기 위치로 이동
    def ready_robot(self):
        movej(JReady, vel=VELOCITY, acc=ACC)
        #gripper.open_gripper()
        mwait()

    # 관찰 위치로 이동
    def move_to_observation_pos(self, target_pos):
        observation_pos = target_pos.copy()
        observation_pos[2] += DETECT_WOUND_DEPTH
        movel(observation_pos, vel=(VELOCITY//2), acc=(ACC//2))
        mwait()
        time.sleep(0.5)
        return observation_pos

    # wound 추적 함수
    def track_wound(self, is_periodic=False):
        fail_count = 0
        max_fail = 5
        while True:
            current_target_pos = self.get_target_pos('wound')
            if current_target_pos is None:
                fail_count += 1
                print(f"절개 부위 인식 실패 {fail_count}회")
                if fail_count >= max_fail:
                    print("절개 부위를 연속으로 인식하지 못했습니다. 추적을 종료합니다.")
                    break
                else:
                    time.sleep(0.5)
                    continue
            else:
                fail_count = 0

            # 주기적 움직임을 위한 시작/끝 위치 계산
            if is_periodic:
                # 먼저 상처 위 70mm 높이로 이동
                observation_pos = current_target_pos.copy()
                observation_pos[2] += DETECT_WOUND_DEPTH
                movel(observation_pos, vel=(VELOCITY//4), acc=(ACC//4))
                mwait()
                
                # 그 위치에서 z축 방향으로 30mm 왕복 운동 (위아래로)
                amp = [0, 0, 30, 0, 0, 0]  # z축 방향으로 30mm
                move_periodic(amp=amp, period=1.0)
            else:
                new_observation_pos = current_target_pos.copy()
                new_observation_pos[2] += DETECT_WOUND_DEPTH
                movel(new_observation_pos, vel=(VELOCITY//4), acc=(ACC//4))
            
            mwait()
            time.sleep(0.5)

            if self.detect_external_force():
                break

    # 오버라이딩으로 코드 재사용
    def pick_and_place_target(self, target_pos, target_type='wound'):
        """
        target_type: 'wound' or 'Yankauer'
        """        
        try:
            release_compliance_ctrl()
            time.sleep(0.5)
            
            # 관찰 위치로 이동
            print("관찰 위치로 이동합니다.")
            self.move_to_observation_pos(target_pos)
            
            # wound 추적 시작
            print("절개 부위 추적을 시작합니다.")
            self.track_wound(is_periodic=(target_type=='Yankauer'))

        except KeyboardInterrupt:
            print("사용자에 의해 추적이 중단되었습니다.")
            self.init_robot()
        except Exception as e:
            print(f"추적 중 오류 발생: {e}")
            self.init_robot()

    # 외력 감지 함수
    def detect_external_force(self):
        task_compliance_ctrl(stx=[480, 480, 480, 180, 180, 180])
        time.sleep(0.1)

        while True:
            if check_force_condition(DR_AXIS_Y, max=self.force_threshold):
                print("외력 감지! 초기 위치로 복귀합니다.")
                release_compliance_ctrl()
                self.init_robot()
                break
            time.sleep(0.5)
        release_compliance_ctrl()

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
                        self.get_logger().info(f"Yankauer position: {YANKAUER_POS}")
                        movec(JReady, YANKAUER_POS, vel=(VELOCITY//2), acc=(ACC//2))
                        mwait()
                        # 그리퍼를 70mm로 열기 (RG2의 최대 너비)
                        gripper.move_gripper(width_val=700)
                        time.sleep(0.2)
                        movel(YANKAUER_GRIP_POS, vel=(VELOCITY//2), acc=(ACC//2))
                        mwait()
                        # 그리퍼를 10mm로 닫기
                        gripper.move_gripper(width_val=10)
                        time.sleep(0.2)
                        movel(YANKAUER_POS, vel=(VELOCITY//2), acc=(ACC//2))
                        mwait()

                        # 2. 수술 부위를 관찰하기 위한 위치로 이동
                        movec(JReady, WAIT_DETECT_WOUND, vel=(VELOCITY//2), acc=(ACC//2))
                        mwait()
                        
                        # 3. wound 찾아서 관찰 및 석션 작업
                        wound_pos = self.get_target_pos("wound")
                        if wound_pos is not None:
                            self.pick_and_place_target(wound_pos, target_type='Yankauer')
                        return
                    except Exception as e:
                        self.get_logger().error(f"Move aborted: {e}")
                        self.init_robot()
                        return
                    
                elif target == "wound":  # wound인 경우에만 모델 인식 수행
                    target_pos = self.get_target_pos(target)
                    if target_pos is None:
                        self.get_logger().warn("No target position")
                    else:
                        self.get_logger().info(f"target position: {target_pos}")
                        try:
                            self.pick_and_place_target(target_pos)
                            return
                        except Exception as e:
                            self.get_logger().error(f"Pick and place aborted: {e}")
                            self.init_robot()
                            return
                else:
                    self.get_logger().warn(f"Unknown target: {target}")
                    return
        else:
            self.get_logger().warn(f"{get_keyword_result.message}")
            return

def main(args=None):
    if not rclpy.ok():
        rclpy.init(args=args)
    
    node = DetectWound()
    while rclpy.ok():
        node.ready_robot()
        node.detect_wound_pos()
        node.robot_control()
        break

    rclpy.shutdown()
    node.destroy_node()


if __name__ == "__main__":
    main()