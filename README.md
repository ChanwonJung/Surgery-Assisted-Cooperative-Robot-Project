# Surgery-assisted cooperative robot project by Using Doosan Manipulator M0609

** Among the above folders, the '최종' folder is the final project source code. **.


## 1. Project Overview
![스크린샷 2025-06-04 13-36-51](https://github.com/user-attachments/assets/6532c598-7d46-4b40-acac-6a35c17f8e70)

- As shown in the following photo, medical gaps such as strikes by minority departments and medical staff have continued to occur.
- It is a surgical-assisted cooperative robot project to reduce the burden on existing medical personnel to help solve this situation even a little.

- The functions are as follows.
### **1) robot_control node**

a. **Anesthesia:** Bring a sleep anesthesia mask to the patient's mouth prior to starting surgery.

b. **Medical device object recognition:****** utilizes STT** to recognize **medical device keywords** within the data label class and picks up when the realsense camera recognizes the object of that keyword.

c. **Hand object recognition:** The realsense camera on the gripper holding the object brings the medical device to the depth where the hand is located when the doctor recognizes the **hand object.

d. When the medical device used by the doctor is recognized by the realsense camera and the medical device in the data label class is recognized, it is repositioned to the place where it was initially placed.

e. **Seal: **Seal at the incised area after recognition of the surgical site (Hemostat) depending on the medical device. **Seal is carried out at the incised area.



### **2) detect_wound node**
   
a. **Expand the surgical incision**: After the manipulator moves to the position where the surgery is taking place, the object at the surgical site ('ex) camera') is recognized through the keyword ('ex) camera').

If it is recognized, it is possible to get a little closer to the area and check the incision that is being operated on the monitor (rqt).
Afterwards, while checking the incision site, when external force is detected through **adaptive control**, the surgery is recognized to be completed and positioned as initial coordinates.


b. **Inhalation of blood using suction**: Using STT(Using Whisper API), inhale blood around the recognized surgical site ('ex) suction, Yankeeur) through keywords.




## 2. Utilization equipment and development environment
- Doosan Manipulator M0609
- ROS2 humble ubuntu 22.04
- OnRobot2 Gripper
- STT (Using Whisper API)
- Surgical Tools: Scalpel, Mayo_metz, Forcep, Hemostat
- Realsense camera
- YOLOv11n

<img width="463" alt="image" src="https://github.com/user-attachments/assets/46f17870-5819-4ee0-8290-3efa13d07690" />






## 3. Project progress
### **- Surgical Tools dataset**
<img width="490" alt="image" src="https://github.com/user-attachments/assets/25375e33-d360-478c-bd98-fa6b3529f29c" />

https://universe.roboflow.com/northeastern-university-ftufl/sgtd

1) Classes label

a. Mayo_metz: Scissors for skin and tissue incision

b. Forceps: Medical tweezers

c. Scalpel: scalpel, small blade

d. Hemostat: arterial clamps, hemostatic devices


2) Use to recognize surgical instruments
   
![image](https://github.com/user-attachments/assets/b49ba1c8-c90a-4586-92c9-239b3509453f)

The model used yolov11n and used epoch=100, img_size=512, and batch=20 (auto) to confirm high-performance object recognition as shown in the picture above.

![image](https://github.com/user-attachments/assets/45044496-10bc-495b-9516-b2bae43d4192)

It was confirmed that the overall loss value decreased as learning progressed, and the precision and recall values exceeded 90%.

### **- Hands dataset**
![image](https://github.com/user-attachments/assets/b0b3b55f-2412-42be-a2d7-b4491cf8a5b2)

https://universe.roboflow.com/hyfyolo/new-hand 

1) Classes used: Hands

2) Use it for hand recognition when gripper picks up and brings the medical device by hand after recognizing it


### **- Surgical Wounds dataset**
![image](https://github.com/user-attachments/assets/59f4c4b4-3c2b-419e-9fbf-9a1801e467da)

https://universe.roboflow.com/myworkspace-zgags/my-first-project-d3ifu/browse?queryText=&pageSize=50&startingIndex=0&browseQuery=true


1) Classes label

a. Stitched

b. Wound


2) Use for incision recognition and suture functions

### -  Flowchart of performance
<img width="494" alt="image" src="https://github.com/user-attachments/assets/d55f09fa-5043-4a30-83af-a6839c123c15" />
<img width="420" alt="image" src="https://github.com/user-attachments/assets/8815abcc-86f5-4730-a8a3-8c9e0ff3b8df" />


<Robot_control algorithm flow chart>

Patient anesthesia sleep progression -> Run wakeword.py via Hello, route -> Use STT to recognize medical device keywords -> Recognize hand objects after holding them -> Deliver medical device to hand when recognized -> Check the presence or absence of medical device put down by medical staff after use -> Return to initial coordinates

<Detect_wound flow chart of the section node>

Hello, run wakeword.py via rokey -> recognize surgical incision keywords using STT -> enlarge surgical site after surgical incision recognition -> return to initial coordinate through compliance control




## 4. Errors experienced during the project
a. The object recognition rate by light reflection has decreased. It is sensitive to the environment.

b. As STT, promports were set to a number of various cases to improve Whisper API performance, but voice recognition was nevertheless not possible.

c. There are a total of 15 classes of data labels, but due to the situation, only 4 classes were used. (Mayo_metz, Forceps, Hemostat, Scalpel)



## 5. Demo

https://drive.google.com/file/d/1ya3sM34Hc5CPN4aMMyc6QHAeQYTwBzs0/view?usp=sharing
