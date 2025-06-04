# Surgery-assisted cooperative robot project by Using Doosan Manipulator M0609

** 위 폴더 중 '최종' 폴더가 최종 프로젝트 소스코드입니다. **

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

<robot_control 알고리즘 순서도>

환자 마취 수면 진행 -> Hello, rokey를 통해 wakeword.py 실행- > STT를 활용하여 의료 기기 키워드 인식 -> 해당 의료 기기 인식 후 잡은 후 손 객체 인식 -> 인식이 되면 손에 의료 기기 전달 -> 의료진들이 사용 후 내려 놓은 의료 기기 유무 확인 -> 초기 좌표 복귀

<절개 부위 노드 detect_wound 순서도>

Hello, rokey를 통해 wakeword.py 실행 -> STT를 활용하여 수술 절개 키워드 인식 -> 수술 절개 인식 후 수술 부위 확대 -> 해당 부위의 수술이 완료되면 -> 순응제어를 통해 초기좌표 복귀




## 4. 프로젝트 중 경험했던 오류들
a. 빛 반사에 의한 객체 인식률이 하락되었다. 환경에 민감하다.

b. STT로써 Whisper API 성능 개선을 위해 다양한 경우의 수로 프롬포트를 설정하였지만 그럼에도 불구하고 음성 인식이 안되는 경우가 발생하였다.

c. 데이터 라벨의 클래스가 총 15개인데 상황 상 어쩔 수 없이 클래스를 4개밖에 활용하지 못하였다. (Mayo_metz, Forceps, Hemostat, Scalpel)



## 5. 시연

