
## jdAMR200 디버깅 전용 Launch 파일 설명서

### 1. 개요

이 문서는 `jdAMR200` 로봇 시스템에서 **`jdamr200_node`만 단독으로 실행**하는 Launch 파일에 대한 설명이다.
이 파일은 전체 시스템 실행이 아닌, **하드웨어 제어 노드 단독 실행 및 디버깅**을 목적으로 사용된다.

주 용도는 다음과 같다:

* **시리얼 통신 연결 확인**
* **IMU/엔코더 데이터 수신 디버깅**
* **`/cmd_vel` 수신 시 모터 동작 확인**
* **ROS2 토픽 발행 상태 테스트**

---

### 2. Launch 파일 역할 요약

| 구성 요소                    | 설명                                                         |
| ------------------------ | ---------------------------------------------------------- |
| `jdamr200_node`          | Arduino 또는 ESP32와 시리얼 통신을 통해 모터, IMU, 엔코더 제어 및 데이터 수신을 수행함 |
| `robot_description` 파라미터 | URDF를 로딩하지만 현재 시각화 노드(RViz 등)가 없어 내부 사용 목적에 가깝다            |
| `use_sim_time`           | 시뮬레이션 시간 사용 여부 설정 가능 (기본값: true)                           |

---

### 3. Launch 파일 소스코드 설명

#### 1) `use_sim_time` 런치 인자 선언

```python
use_sim_time = LaunchConfiguration('use_sim_time', default='false')
```

* 시뮬레이션 시간(`clock` 토픽) 사용 여부를 결정하는 파라미터.
* 실제 하드웨어 테스트 환경에서는 일반적으로 `false`로 설정함.

#### 2) URDF 경로 읽기

```python
jdamr200_urdf = os.path.join(
    get_package_share_directory('jdamr200_description'),
    'urdf',
    'jdamr200.urdf')
with open(jdamr200_urdf, 'r') as file:
    jdamr200_desc = file.read()

robot_param = {'robot_description': jdamr200_desc}
```

* `jdamr200_description` 패키지의 URDF 파일을 불러와 파라미터로 설정.
* 현재 이 파라미터는 사용되지 않고 있으며, RViz 등의 시각화 노드가 없기 때문에 실질적 효과는 없음.

#### 3) `jdamr200_node` 실행 노드 정의

```python
Node(
    package='jdamr200_node',
    executable='jdamr200_node',
    output='screen',
)
```

* ROS2 노드 `jdamr200_node`를 실행.
* 이 노드는 다음 기능을 수행함:

  * `/cmd_vel` 수신 시 직렬 통신으로 모터 제어 명령 전송
  * IMU 센서 및 바퀴 엔코더 값을 수신하여 ROS 토픽으로 발행 (예: `/odom`, `/imu`)
  * 전압 정보 `/Voltage` 등 내부 상태 확인용 토픽도 발행 가능

---

### 4. 사용 목적 및 테스트 사례

이 Launch 파일은 전체 시스템을 실행하지 않고, **하드웨어 연동 확인 또는 개발 중 디버깅 목적**으로 사용된다. 다음과 같은 테스트에 유용하다:

* **USB 포트 확인 및 권한 설정 후 노드 실행**
* **`/cmd_vel` 토픽 발행 테스트 → 로봇 움직임 확인**
* **IMU/엔코더 데이터 수신 여부 확인 (`rqt_graph`, `rqt_topic`)**
* **전원 전압 확인 (`/Voltage` 확인)**

---

### 5. 실행 방법

```bash
source ~/jdamr200_ws/install/setup.bash
ros2 launch jdamr200_bringup jdamr200_node_launch.py
```

---

### 6. 디버깅 예시

1. `jdamr200_node` 실행 후 아래 명령으로 토픽 확인:

   ```bash
   ros2 topic list
   ```

2. `/cmd_vel`을 발행해 모터 제어 확인:

   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5, y: 0.0, z: 50.0}, angular: {z: 0.0}}"
   ```

3. IMU나 엔코더 확인:

   ```bash
   ros2 topic echo /odom
   ros2 topic echo /imu
   ```

---

### 7. 결론

이 Launch 파일은 `jdamr200_node`를 단독으로 실행하여 하드웨어 동작 여부를 확인하거나 개발 중 개별 모듈 테스트를 할 때 유용하다.
전체 SLAM 시스템 또는 자율주행 시스템과 별개로 작동하기 때문에 **ROS2 기반 로봇 시스템 개발에서 매우 중요한 기초 단계 디버깅 도구**로 활용할 수 있다.
