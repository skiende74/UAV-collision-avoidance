### UAV 경로회피 알고리즘. 
다수의 UAV 운용시 아군 UAV간 충돌을 예측, 회피경로작성, 회피, 기존미션경로복귀를 담당하는 알고리즘.
프로젝트를 해결하기위해 논문을 서치, MATLAB으로 검증, ROS C++상으로 점진적 이동 및 문제점 개선 수행

아이디어를 프로젝트 조건에 맞게 변형하여 차용한 논문
- [**"A Fast Obstacle Collision Avoidance Algorithmfor Fixed Wing UAS" - Zijie
Lin, Lina Castano, and Huan XuMember, IEEE**](https://user.eng.umd.edu/~mumu/files/LCX_ICUAS2018.pdf)

#### 과제 조건
- UAV는 인접 UAV와의 충돌위험성을 스스로 판단하고 회피
- 이론적으로 확실한 회피가 보장되어야함.
- 고정익과 회전익에 모두적용가능해야함.
- UAV기체의 spec(최소회전반경 rho, 항속 v0)가 반영되어야함.
- 최대한 단순하고, 최소한의 변화로 회피하여야 함.
- 회피상황 발생시, 좌측에 있는 UAV가 우측의 UAV를 우향회피하여야함 (국제회피규약)
- 회피이후 본래의 MISSION경로로 복귀하여 미션을 재수행

### 기존 리서치 조사 & 원래 되어있던 것.
원래 기존에 연구실에서는 Potential Flow (드론의 움직임을 유체역학적 유동으로 해석(비유)하여 회피)를 
적용해보고 있었으나 잘 작동하지 못한상황이었음.

ROS C++상에서 MAVLink(통신) API를 받아들이는 등의 기본코드는 되어있었음.

기존에 박사연구생이던 선배가 논문을 2건 정도 찾아놓으셨음.

### 직접 진행한 것
적절한 논문선택, MATLAB으로 실현가능성검증, ROSC++/Gazebo환경으로 점진적 이동 및 문제점개선 등
경로회피 관련 모든 작업은 혼자 진행하였음.
(  제가 진행사항을 보고하면 박사연구생 선배님이 들어주셨고. 
MATLAB으로 먼저 해보고 ROS C++로 옮기라는 조언.
문제상황을 캐치하였을때 디스커션에 참여해주신 등의 도움을 받았습니다. )

박사연구생 선배가 찾아두었던 2개의 논문과 스스로찾은 1개의 논문 중
**"A Fast Obstacle Collision Avoidance Algorithmfor Fixed Wing UAS" - Zijie
Lin, Lina Castano, and Huan XuMember, IEEE**
논문이 단순성, 효율성이 좋다고 판단하여 이 논문의 아이디어를 적절히 차용하기로 채택하였음.


### 진행한 연구 순서
MATLAB으로 원시코드를 우선 작성하여 논문의 재현가능성을 검증하고,
ROS/Gazebo환경으로 코드를 점진적으로 이동함.

#### 논문의 알고리즘을 현 프로젝트에 적용시 발생하였던 문제 및 그 해결
- FGA 알고리즘의 가정은 좌우회피 가능하나, 이번 프로젝트에서는 우향회피를 하여야하기 때문에,
  이를 고치면 이론적 회피보장이 깨짐. -> 실좌표계에서 원형궤적으로 회피하게끔 개선.
- 상대좌표계에서 등속가정이 깨짐 -> 절대좌표계에서 등속이게끔 함.
- 실제 속도의 방향이 변함에따라 상대속도의 크기가 변함 -> 미리 어느정도 상대속도회전에따른 크기변화 예상중 가장 불리한 예측결과를사용하도록 하여 해결

### 간단한 설명
![image](https://github.com/skiende74/UAV-collision-avoidance/assets/86130706/4c95e342-c15a-4631-a454-df4ffe1eeea1)
위 원 궤도를 따르면서, 상대UAV와 d_m만큼의 거리를 유지하며 최소회피 합니다.

### 다양한 상황에서의 회피결과
파란 그래프는 내 UAV와 상대 UAV와의 거리를 나타냅니다.
![image](https://github.com/skiende74/UAV-collision-avoidance/assets/86130706/9023716f-a697-4552-aedb-29795cabec1a)
![image](https://github.com/skiende74/UAV-collision-avoidance/assets/86130706/52b7d399-4df7-4170-bb10-9297837b590f)
![image](https://github.com/skiende74/UAV-collision-avoidance/assets/86130706/d1de6f9f-efc3-484b-9722-0d81e17a2216)
![image](https://github.com/skiende74/UAV-collision-avoidance/assets/86130706/dc70d2ae-21e3-4d5b-9ddd-675a2a1ed529)


이 설명은 축약된 것이며 상세 내역은
[report](https://github.com/skiende74/UAV-collision-avoidance/blob/main/report.pdf) 에 있습니다.
