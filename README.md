# jdAMR200 ROS2 패키지 

## 📁 Repository Structure

```
jdamr200_ros/
├── jdamr200_description/      # URDF 기반 로봇 모델 패키지 (RViz 시각화 용도)
├── jdamr200_node/             # 임베디드 보드(Arduino/ESP32)와의 통신 노드
├── jdamr200_teleop/           # 키보드 입력을 받아 cmd_vel 발행 (teleop 노드)
├── jdamr200_bringup/          # 여러 노드를 실행하기 위한 launch 파일들
├── jdamr200_cartographer/     # Cartographer 기반 SLAM 패키지
└── lidar_sl_ros2/             # LP14 LDlidar ROS2 드라이버 (외부 제공)
```

### 🔧 패키지 설명

* **`jdamr200_description`**
  URDF 파일을 통해 jdAMR200 로봇의 시각적 모델을 정의합니다. RViz에서 사용됩니다.

* **`jdamr200_node`**
  Arduino 또는 ESP32와 직렬 통신을 통해 실제 하드웨어를 제어합니다.

  * **기능**:

    * `/cmd_vel` 토픽 수신 → 모터 제어
    * IMU, 엔코더 데이터 수신 및 ROS 토픽으로 발행

* **`jdamr200_teleop`**
  키보드 입력을 받아 `/cmd_vel` 메시지를 발행하는 노드입니다.

  * 키보드로 로봇을 원격 조종할 수 있습니다.

* **`jdamr200_bringup`**
  실행 편의를 위한 launch 파일을 포함합니다.

  * URDF 모델을 RViz에 로딩
  * Lidar 및 통신 노드 실행 등 자동화

* **`jdamr200_cartographer`**
  2D SLAM을 위한 Cartographer 구성을 포함합니다.

  * Lidar 데이터를 이용해 자율주행 맵 생성 가능

* **`lidar_sl_ros2`**
  LP14 LDlidar용 ROS2 드라이버 패키지입니다.

  * 외부에서 제공된 드라이버를 포함

---

## 🚀 Installation

```bash
# 1. 워크스페이스 생성
mkdir -p ~/jdamr200_ws/src
cd ~/jdamr200_ws/src

# 2. 본 레포지토리 복제
git clone https://github.com/JD-edu/jdamr200_ros.git .

# 3. 의존성 설치 (선택 사항)
rosdep install --from-paths . --ignore-src -r -y

# 4. 빌드
cd ~/jdamr200_ws
colcon build

# 5. 환경 설정
source install/setup.bash
```

---

## ⚙️ 사용 방법

### 1. 모터 제어 및 키보드 텔레옵 실행

```bash
# 터미널 1: 하드웨어 통신 노드 실행
ros2 run jdamr200_node jdamr200_node

# 터미널 2: 키보드 조작으로 cmd_vel 발행
ros2 run jdamr200_teleop jdamr200_teleop
```

* `jdamr200_node`는 `/cmd_vel`을 수신하여 로봇의 바퀴를 제어합니다.
* `jdamr200_teleop`은 키보드 입력을 받아 `/cmd_vel`을 발행합니다.

### 2. RViz에서 로봇 모델 시각화

```bash
ros2 launch jdamr200_bringup display.launch.py
```
