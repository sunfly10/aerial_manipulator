# 🚁 Aerial Manipulator: 창고 환경 내 비전 기반 자율 파지 드론 시스템

<img width="342" height="306" alt="Screenshot from 2026-06-29 15-12-56" src="https://github.com/user-attachments/assets/452a126a-6852-4815-bdc7-3017cff3d323" />
| ARMING STATE | TAKE OFF STATE |
| :---: | :---: |
| <img width="3662" height="1988" alt="Screenshot from 2026-06-29 15-21-58" src="https://github.com/user-attachments/assets/75629baf-bcfb-442b-9c46-81c3831a1d37" /> | <img width="3662" height="1988" alt="image" src="https://github.com/user-attachments/assets/b52ff22b-bbb7-41dc-8ff5-e86d94f0857a" /> |

| TRACKING STATE | WAITING STATE |
| :---: | :---: |
| <img width="3662" height="1988" alt="image" src="https://github.com/user-attachments/assets/97a6de0f-be00-4e46-87ff-49ca123f3ef1" /> | ![Uploading Screenshot from 2026-06-29 15-27-49.png…]() |


파지 알고리즘 구체화 진행중

---

## 1. 프로젝트 개요

본 프로젝트는 산업용 물류 창고 환경을 모사한 Gazebo 시뮬레이션 내에서, 로봇 팔(Manipulator)과 Body-mounted Downward Camera를 장착한 쿼드롭터(x500)를 이용하여 목표물을 탐색, 정렬, 그리고 파지하는 시스템을 구축한 개인 프로젝트입니다.

단순 비행 제어를 넘어, 로봇 팔의 길이와 카메라 장착 위치에서 발생하는 기구학적 오프셋을 역산 캘리브레이션으로 극복하고, 개방 루프(Open-loop) 제어의 불확실성을 없애기 위해 비전과 촉각 데이터를 실시간 피드백으로 활용하는 고신뢰성 자율 파지 아키텍처를 설계했습니다.

### 🛠️ 개발 환경 및 기술 스택
* **OS / Middleware:** Ubuntu 24.04 (Noble), ROS 2 (Jazzy), Micro-XRCE-DDS
* **Simulation:** PX4 Autopilot (v1.14+), Gazebo Harmonic
* **Language:** Python 3
* **Core Tech:** OpenCV (`solvePnP`), Visual Servoing (EMA Filter), Kinematics Offset Calibration, FSM (Finite State Machine)

---

## 2. System Architecture

무거운 물리 엔진 연산과 비전 처리, 실시간 비행 제어 로직 간의 통신 지연(Latency)을 최소화하기 위해 통합 제어 아키텍처를 설계했습니다.

* **Simulation (Gazebo & PX4):** 물류 창고 환경, ArUco 마커, 드론의 동역학적 비행 및 접촉 센서(Contact Sensor)의 물리 연산 수행.
* **Bridge (`ros_gz_bridge`):** YAML 설정 파일을 통해 Gazebo의 고해상도 영상 및 센서 데이터를 ROS 2 메시지로 변환하여 양방향 라우팅.
* **Control Node (`autonomous_control.py`):** 카메라 영상 수신 및 PnP 연산(Perception), FSM 기반 비행 제어(Control), 로봇 팔 관절 제어(Actuation)를 하나의 노드에서 All-in-One으로 통합 처리하여 제어 주기 최적화.

---

## 3. 개발 프로세스

시스템의 안정성을 확보하기 위해 아래 3단계의 개발 프로세스를 거쳐 완성되었습니다.

* **Phase 1: 동역학 테스트 및 수동 제어 (`keyboard_control.py`)**
  * 위치/속도 혼합 제어 모드를 구현하여 비상시 특정 고도에서 정지(Hovering)하는 로직을 테스트했습니다.
* **Phase 2: 비전 캘리브레이션 및 오차 역산 (`perception_arukomarker.py`)**
  * 가제보 물리 엔진의 절대 거리와 PnP 추정 수학적 거리가 완벽히 일치할 때까지 비례식을 적용하여 맞춤형 카메라 파라미터를 튜닝한 비전 전담 노드입니다.
* **Phase 3: All-in-One 통합 자율 제어 (`autonomous_control.py`)**
  * 비전 인식부터 자율 비행, 로봇 팔 파지(Close-until-Contact), FSM 제어까지 전 과정을 하나의 노드로 통합 수행하여 통신 지연을 최소화했습니다.
  * 파지 제어 알고리즘 검증 중

---

## 4. 설치 및 의존성

프로젝트 실행을 위한 미들웨어 및 패키지 설치는 각 공식 GitHub 레포지토리의 가이드를 참조함.
1. [PX4-Autopilot GitHub](https://github.com/PX4/PX4-Autopilot)
2. [Micro-XRCE-DDS-Agent GitHub](https://github.com/eProsima/Micro-XRCE-DDS-Agent)
3. [px4_msgs](https://github.com/PX4/px4_msgs) & [px4_ros_com](https://github.com/PX4/px4_ros_com) (ROS 2 워크스페이스 내 빌드 필요)

---

## 5. 실행 방법

성공적인 실행을 위해 모든 터미널 창에서 공통적으로 아래의 환경 변수 및 소스 파일 적용이 선행되어야 합니다.

* 모든 터미널 공통 실행

`bash`

`source /opt/ros/jazzy/setup.bash`

`source ~/drone_ws/install/setup.bash`

`export GZ_IP=127.0.0.1`

`export GZ_PARTITION=default`

* Terminal 1: 통신 에이전트 실행

`bash`

`cd ~/Micro-XRCE-DDS-Agent/build # Micro-XRCE-DDS-Agent 설치 폴더로 이동 후 실행`

`./MicroXRCEAgent udp4 -p 8888`

* Terminal 2: PX4 SITL 및 시뮬레이션 환경 실행

`bash`

`cd ~/PX4-Autopilot`

`make px4_sitl gz_x500_depth`

* Terminal 3: ROS 2 - Gazebo 브릿지 실행

`bash`

`ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=bridge.yaml # YAML 파일을 사용하여 카메라 및 로봇팔 토픽을 일괄 매핑합니다(yaml 파일이 있는 곳에서 실행).`

* Terminal 4: 메인 자율 제어 노드 실행

`autonomous_control.py` 실행

---

## 6. Key Technologies & Control Logic

### 6.1 물리적 오차 보정 역산 캘리브레이션
카메라 렌즈 표면과 드론 무게중심 간에는 로봇 팔 길이(0.41m)와 랜딩 기어(0.20m)로 인한 물리적 오프셋이 존재합니다. 
* **해결 방안:** 가제보 물리 엔진의 절대 거리 데이터와 카메라가 인식하는 수학적 거리가 완벽히 일치할 때까지 비례식을 사용하여 본 기체만의 맞춤형 초점거리(Fx, Fy = 1435.5)를 역산 도출하여 시스템에 적용했습니다.

### 6.2 시각적 서보잉 (Visual Servoing) 및 속도 제어
하향 카메라를 이용해 ArUco 마커의 3D 좌표를 추정하고 픽셀 오차를 최소화합니다.
* **필터링 적용:** 산출된 픽셀 오차와 목표 거리 오차에 선형 제어와 지수이동평균(EMA) 필터를 적용하여, 바람이나 관성에 의한 드론의 흔들림을 보정하고 부드러운 하강 궤적($v_x$, $v_y$, $v_z$)을 생성했습니다.

### 6.3 촉각 센서 기반 파지 (Close-until-Contact)
단순 지정 고도에서 그리퍼를 닫는 맹목적 방식의 실패율을 낮추기 위해 반응형 제어를 도입했습니다.
* **접촉 피드백:** 로봇 팔 양쪽 손가락에 Contact Sensor를 부착하여, 하강 중 목표물에 닿는 즉시(`left_contacted`, `right_contacted` True 반환) 하강을 중단하고 즉각적인 그리퍼 파지(Action) 상태로 돌입합니다.

### 6.4 유한 상태 머신 (FSM) 기반 임무 무결성 확보
비행부터 파지 후 복귀까지의 복잡한 시나리오를 상태(State)로 나누어 순차적으로 제어합니다.
* `ARMING` ➡️ `TAKEOFF` ➡️ `TRACKING` (마커 추종) ➡️ `WAITING` (정렬 안정화) ➡️ `ALIGN_BOX` (정밀 조준) ➡️ `DESCENDING` (하강) ➡️ `ACTION` (센서 기반 파지) ➡️ `ASCENDING` (상승) ➡️ `DONE`

---

## 7. 트러블슈팅 및 고찰

### 💡 기구학적 오차로 인한 Crash 문제 해결
* **Issue:** 초기 하강 제어 시, 드론이 목표 상자를 파지하지 못하고 바닥에 충돌하며 전복되는 현상이 발생했습니다.
* **Solution:** 코드상의 고도(`current_alt`)가 땅바닥 기준이 아닌 랜딩 기어를 포함한 드론 몸체 기준임을 파악했습니다. 로봇 팔 길이(0.41m)와 랜딩 기어(0.20m)를 동역학적으로 계산하고, 실제 시뮬레이션 환경에서의 반복적인 파라미터 튜닝을 거쳐 `목표 상대 고도 = 0.25m`가 가장 최적의 안전 파지 고도임을 도출하여 충돌 문제를 해결했습니다.

---

## 8. 한계점 및 향후 과제

본 프로젝트의 체계 통합 과정을 통해 시뮬레이션 물리 엔진의 한계와 동역학적 변수를 인지하고 다음과 같은 개선 소요를 도출했습니다.

* **가제보 접촉 마찰(Friction) 한계 극복:** 현재 그리퍼가 닫힐 때 물리 엔진의 마찰 계수 및 질량비 문제로 상자가 미끄러지는 현상이 발생합니다. 향후 그리퍼의 형태를 V자 홈 등으로 개선하고 물리 엔진 접촉 파라미터를 세밀하게 튜닝할 계획입니다.
* **Payload 추가에 따른 동역학 보상 제어:** 파지 후 상자의 무게(Payload)가 추가되면 드론의 무게 중심이 변하여 피치(Pitch) 흔들림이 발생합니다. 화물 적재 여부에 따라 PID 게인 값을 동적으로 조절하거나 무게를 선제적으로 보상하는 피드포워드(Feed-forward) 제어 로직 도입이 필요합니다.
* **비전 노이즈 고도화:** 근접 하강 시 카메라 화각 제한으로 인해 발생하는 바운딩 박스 흔들림(Jittering)을 보정하기 위해 칼만 필터(Kalman Filter) 기반의 위치 추정 알고리즘을 도입할 예정입니다.

---

9. 참고 및 출처

Warehouse Model: 원작자 Filipe Almeida (mov.ai)의 오픈소스 창고 모델을 기반으로 시뮬레이션 환경 구성.

ArUco Marker Model: Jacob Dahl (ArkElectron)의 오픈소스 모델을 기반으로 드론 내비게이션용 커스텀 ID 모델 수정 및 확장 활용.
