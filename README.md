# Surgery-assisted cooperative robot project by Using Doosan Manipulator M0609

** 위 폴더 중 ( ) 폴더가 최종 프로젝트 소스코드입니다. **

## 1. 프로젝트 개요
![스크린샷 2025-06-04 13-36-51](https://github.com/user-attachments/assets/6532c598-7d46-4b40-acac-6a35c17f8e70)

- 다음 사진과 같이 최근 소수과나 의료진 파업 등 의료 공백이 지속적으로 발생하고 있다.
- 이러한 사태를 조금이나마 해결하는데 있어 도움이 되고자 기존 남아있는 의료인들의 부담을 줄여주기 위한 수술 보조 협동 로봇 프로젝트이다.

- 기능으로는 아래와 같다.
### **1) robot_control node**

   a. **마취:** 수술에 시작하기 앞서 환자의 입 쪽으로 수면 마취 마스크를 가져다 준다.

   b. **의료기기 객체 인식:** **STT**를 활용하여 데이터 라벨 클래스 내 **의료 기기 키워드**를 인식하고 realsense camera가 해당 키워드의 객체를 인식하면 집는다.

   c. **손 객체 인식:** 객체를 집고 있는 그리퍼 위에 있는 realsense camera가 의사의 **hand 객체 인식** 시 hand가 위치해있는 depth로 의료기기를 가져다 준다.

   d. 의사가 사용하고 놓은 의료 기기를 realsense camera가 인식하여 데이터 라벨 클래스 내 의료기기가 인식되면 기존 초기에 놓여져 있는 곳으로 다시 위치한다.

   e. **봉합:** 의료기기에 따라 수술 부위 인식 후 (Hemostat) 절개된 부위에서 **봉합**을 실시한다.



### **2) detect_wound node**
   
   a. **수술 절개 부위 확대**: 수술이 진행되고 있는 위치로 매니퓰레이터가 움직인 후 STT를 활용하여 키워드를 통해 ('ex) 카메라') 수술 부위(wound)의 객체를 인식한다.

   인식이 되었다면 해당 부위로 조금 더 다가가 모니터(rqt)상으로 수술하고 있는 절개 부위를 확인할 수 있다. 
   이후 절개 부위 확인하는 동안 **순응 제어**를 통해 외력 감지 시 수술이 완료되었다고 인식하고 초기좌표로 위치한다.


   b. **석션을 활용한 혈액 흡입**: STT를 활용 하여 키워드를 통해 ('ex) 석션, Yankeur) 인식된 수술 부위(wound) 주변 혈액을 흡입한다.





## 2. 활용 장비 및 개발환경
- Doosan Manipulator M0609
- ROS2 humble ubuntu 22.04
- OnRobot2 Gripper
- STT (Using Whisper API)
- Surgical Tools: Scalpel, Mayo_metz, Forcep, Hemostat
- Realsense camera
- YOLOv11n

<img width="463" alt="image" src="https://github.com/user-attachments/assets/46f17870-5819-4ee0-8290-3efa13d07690" />






## 3. 프로젝트 수행 경과
### **- Surgical Tools dataset**
<img width="490" alt="image" src="https://github.com/user-attachments/assets/25375e33-d360-478c-bd98-fa6b3529f29c" />
https://universe.roboflow.com/northeastern-university-ftufl/sgtd

1) 사용한 클래스

   a. Mayo_metz: 피부 및 조직 절개용 가위

   b. Forceps: 의료용 핀셋

   c. Scalpel: 메스, 소형 칼날

   d. Hemostat: 동맥 집게, 지혈기


2) 수술 도구 인식에 활용
   
![image](https://github.com/user-attachments/assets/b49ba1c8-c90a-4586-92c9-239b3509453f)
모델은 yolov11n을 활용하였고 epoch=100, img_size=512, batch=20 (이하 auto)를 활용하여 위 사진과 같이 높은 성능을 띄는 객체 인식 확인이 가능하였다.

![image](https://github.com/user-attachments/assets/45044496-10bc-495b-9516-b2bae43d4192)
학습이 진행됨에 따라서 전체적인 loss 값이 감소하고 precision, recall 값이 90% 이상 넘기는 것을 확인할 수 있었다.

### **- Hands dataset**
![image](https://github.com/user-attachments/assets/b0b3b55f-2412-42be-a2d7-b4491cf8a5b2)
https://universe.roboflow.com/hyfyolo/new-hand 


1) 사용한 클래스: Hands
   
2) 그리퍼가 의료기기를 인식한 후 의료기기를 집어 손으로 가져다 줄 때 손 인식에 활용


### **- Surgical Wounds dataset**
![image](https://github.com/user-attachments/assets/59f4c4b4-3c2b-419e-9fbf-9a1801e467da)
https://universe.roboflow.com/myworkspace-zgags/my-first-project-d3ifu/browse?queryText=&pageSize=50&startingIndex=0&browseQuery=true


1) 사용한 클래스

   a. Stitched(실밥으로 꿰멘 자국)
   b. Wound(흉터 절개)


2) 절개 부위 인식 및 봉합 기능에 활용

### - 수행 흐름도
<img width="494" alt="image" src="https://github.com/user-attachments/assets/d55f09fa-5043-4a30-83af-a6839c123c15" />
<img width="420" alt="image" src="https://github.com/user-attachments/assets/8815abcc-86f5-4730-a8a3-8c9e0ff3b8df" />


![detected wound(1)](https://github.com/user-attachments/assets/a9284924-59c4-4c69-9646-707202e4180c)

![robot control](https://github.com/user-attachments/assets/896c4ee4-02dc-456d-8046-ea6a0bfaeabe)

<알고리즘 순서도>: 공장에서 허브로 블록 입고 -> 허브 안 영역에서 블록 적재 -> 소비자로부터 주문이 들어왔을 때 블록 출고


a. 블록 입고: 컨베이어 벨트로 블록이 접근하면 매니퓰레이터가 z축으로 하강 한 이후 물체를 잡는다. 이후 task_compliance_ctrl(), set_desired_force()로 순응 제어 및 외력을 통해 허브 안으로 블록을 적재한다.


b. 블록 적재: 블록을 적재할 때 레고가 구멍에 잘 들어갈 수 있도록 외력을 인식하게 설정하였다. 이후 블록이 안정화되면 성공으로 간주하고 매니퓰레이터는 초기 좌표로 이동한다. 블럭 크기별로 한번에 최대 3층까지 쌓을 수 있게 설정하였고 크기별로 허용 가능한 허브 영역 내 다 쌓았는데도 불구하고 적재해야 할 블록이 입고된다면 오버플로우 영역에 크기 상관없이 최대 4개까지만 적재할 수 있도록 설정하였다.


![image](https://github.com/user-attachments/assets/020dd59c-7ca9-48b7-9e8d-478843161db8)
블록이 적재될 때 순간적으로 z축에 외력이 생기는 모습을 확인할 수 있다.

![image](https://github.com/user-attachments/assets/4bd6861c-61c8-473a-b3e4-da4e09364a2f)

<img width="669" alt="image" src="https://github.com/user-attachments/assets/57522522-255f-48f8-bab9-ebbec209afdd" />


c. 블록 출고: 소비자로부터 주문이 들어와 허브 내에 있는 상품을 출고해야 할 때 그리퍼가 해당 상품을 잡아 Ry축으로 한번 비튼 후 z축으로 상행한다. 이후 출고 좌표로 해당 상품을 놓고 이후 다시 초기좌표로 이동한다.


d. 불량품 처리: 컨베이어 벨트에서 그리퍼로 상품을 집었을 때 불량품으로 인식된다면 불량품으로 처리되는 자체 영역에 따로 보관한다. 이후 get_current_posx()[2]로 불량품 영역에 쌓여있는 불량품들의 높이를 측정 후 불량품들이 조금은 평탄하게 쌓일 수 있도록 move_periodic() 함수를 사용하였다.



## 4. 프로젝트 중 경험했던 오류들
a. weblogic이 아닌 onrobot 그리퍼가 해당 물체를 집었을 때 그리퍼의 너비를 실시간으로 통신하는 PyModBus Protocol을 활용하려 하였지만 I/O controller 특성 상 그러지 못하였다.
https://github.com/ABC-iRobotics/onrobot-ros2/tree/main/onrobot_rg_control/onrobot_rg_control

b. set_desired_force() 외력 명령에 movel()을 같이 사용하여 하드웨어 자체의 이벤트가 거절되었다.



## 5. 시연

https://drive.google.com/file/d/1xDryZHY2w0bPtmrQmrJiEwY50bOkdlcB/view?usp=drive_link
