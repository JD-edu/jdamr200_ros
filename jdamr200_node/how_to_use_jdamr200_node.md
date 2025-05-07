# jdAMR200_node 사전 설정, 실행, 소스 설명 

## 🔌 1. jdamr200 임베디드 보드 시리얼 포트 확인 방법

### ✅ 시리얼 포트 목록 확인

터미널에서 아래 명령어를 입력하면 연결된 시리얼 디바이스를 확인할 수 있습니다:

```bash
$ ls /dev/tty*
```

### ✅ 자주 사용되는 포트 이름 예시

* Arduino: `/dev/ttyACM0` 또는 `/dev/ttyUSB0`
* ESP32: `/dev/ttyUSB0` 또는 `/dev/ttyUSB1`

보드를 USB로 연결한 직후 새롭게 생긴 `/dev/tty*`가 해당 포트입니다.

---

## 🔐 2. 시리얼 포트 권한 설정

시리얼 포트는 일반 사용자에게 기본적으로 접근 권한이 없을 수 있습니다.

### ✔ 방법 1: `chmod`로 직접 권한 부여

```bash
$ sudo chmod 777 /dev/ttyACM0
```

> 위 명령은 해당 포트에 읽기/쓰기 권한을 부여합니다. 다만 USB를 재연결하면 초기화되므로 매번 실행해야 할 수 있습니다.

### ✔ 방법 2: 사용자 그룹에 추가 (추천)

```bash
$ sudo usermod -a -G dialout $USER
```

그 후 로그아웃 → 다시 로그인하면 `/dev/ttyACM0` 접근 권한이 자동으로 주어집니다.

---

## 🧠 jdamr200\_node.py 코드 설명

이 노드는 `Jdamr200`이라는 클래스(별도 라이브러리 파일에 구현됨)를 통해 **임베디드 보드와 시리얼 통신**을 합니다. 주요 기능은 다음과 같습니다:

### 📥 `/cmd_vel` 구독

* **키보드 조작 또는 ROS 제어 노드**에서 발행된 `/cmd_vel` 메시지를 수신합니다.
* 메시지의 linear.x/y/z 값을 기반으로 로봇의 방향과 속도를 결정하여, `Jdamr200.move_run_mode()`를 통해 보드로 모터 제어 명령을 전송합니다.

### 📤 `/odom` 퍼블리시

* 일정 주기(0.05초)로 `self.robot.readSpeed()`와 `update_odometry()`를 호출해 로봇의 위치를 계산하고, ROS `Odometry` 메시지로 퍼블리시합니다.

### 🔋 `/Voltage` 퍼블리시

* 1초마다 임의의 전압값(예: 100)을 발행합니다. (실제 전압 측정 기능은 향후 구현될 수 있습니다)

---

## 🏁 실행 방법 요약

```bash
# 소스워크스페이스 로딩
source ~/jdamr200_ws/install/setup.bash

# jdamr200_node 실행
ros2 run jdamr200_node jdamr200_node
```

---

이 노드는 하드웨어에 직접 명령을 내리는 역할을 하므로, 실행 전에 반드시:

1. 시리얼 포트 확인 (`ls /dev/tty*`)
2. 시리얼 권한 부여 (`chmod` 또는 `dialout` 그룹 추가)
3. `Jdamr200` 클래스에서 사용하는 포트(`/dev/ttyACM0` 등)가 정확히 설정되어 있는지 확인

위 항목을 점검해 주세요.

## 3 노드 소스코드 설명 

좋습니다. 아래는 `jdamr200_node`의 핵심 Python 소스코드(`jdamr200_node.py`)에 대한 **중간 수준의 상세 설명**입니다. 전체적인 흐름과 각 주요 구성 요소의 역할을 이해할 수 있도록 주석 중심으로 설명드리며, 코드의 세부 구현은 간단히 요약합니다.

---

## 🧠 전체 구조 요약

이 노드는 `Jdamr200`이라는 클래스 인스턴스를 통해 **시리얼 통신으로 로봇을 제어**합니다. 동시에 ROS2에서 **다양한 퍼블리셔와 서브스크라이버**를 설정하여 모터 제어, 오도메트리 퍼블리싱, 배터리 전압 확인 등의 기능을 수행합니다.

---

## 📄 주요 코드 설명

### 1. 클래스 정의 및 초기화

```python
class JdamrControlNode(Node):
    def __init__(self):
        super().__init__('jdamr_control_node')
```

* ROS2 노드를 초기화합니다. 노드 이름은 `'jdamr_control_node'`입니다.

```python
self.robot = Jdamr200("/dev/ttyACM0")
```

* `Jdamr200` 객체를 생성합니다. `/dev/ttyACM0`는 아두이노나 ESP32가 연결된 **시리얼 포트**입니다.
* 실제 모터 제어 및 센서 데이터 수신은 이 클래스에서 수행됩니다.

---

### 2. 퍼블리셔 및 서브스크라이버 생성

```python
self.publisher_ = self.create_publisher(String, 'jdamr_control', 10)
```

* 테스트용 문자열 퍼블리셔 (`jdamr_control` 토픽, 사용되지 않음)

```python
self.pub_odom = self.create_publisher(Odometry, 'odom', 50)
```

* `nav_msgs/Odometry` 메시지를 `odom` 토픽으로 퍼블리시합니다.
* 위치 및 방향 데이터를 SLAM, Navigation 등에서 사용할 수 있도록 전달합니다.

```python
self.pub_v = self.create_publisher(Int8, 'Voltage', 1000)
```

* 배터리 전압(전류)을 표현하기 위한 퍼블리셔입니다. 현재는 값 `100`만 퍼블리시합니다.

```python
self.subscriber_cmd_vel = self.create_subscription(
    Twist,
    '/cmd_vel',
    self.cmd_vel_callback,
    10
)
```

* `Twist` 메시지를 수신하여 로봇을 제어합니다. 키보드 또는 자율주행 노드에서 발행됩니다.

---

### 3. 타이머 등록

```python
self.odom_timer = self.create_timer(0.05, self.odom_timer_callback)
```

* 0.05초마다 오도메트리 정보를 갱신하고 발행합니다.

```python
self.volt_timer = self.create_timer(1, self.volt_timer_callback)
```

* 1초마다 전압 데이터를 발행합니다.

---

### 4. 오도메트리 콜백

```python
def odom_timer_callback(self):
    msg = Odometry()
    msg.header.frame_id = 'odom'
```

* 오도메트리 메시지를 생성합니다.

```python
self.robot.readSpeed()
result = self.robot.update_odometry()
```

* `readSpeed()`: 현재 엔코더 속도값을 읽습니다.
* `update_odometry()`: 속도와 시간 정보를 바탕으로 x, y, θ를 계산합니다.

```python
msg.pose.pose.position.x = result[0]
msg.pose.pose.position.y = result[1]
msg.pose.pose.position.z = result[2]
self.pub_odom.publish(msg)
```

* 위치 값을 메시지에 저장하고 퍼블리시합니다.

---

### 5. 전압 콜백

```python
def volt_timer_callback(self):
    msg = Int8()
    msg.data = 100
    self.pub_v.publish(msg)
```

* 현재는 전압 값을 `100`으로 고정해서 보내지만, 실제 로봇에서는 배터리 센서를 연결해 값을 갱신할 수 있습니다.

---

### 6. `/cmd_vel` 콜백

```python
def cmd_vel_callback(self, msg):
    go_back = msg.linear.x
    rotate = msg.linear.y
    speed = int(msg.linear.z)
```

* ROS2의 `geometry_msgs/Twist` 메시지에서 이동 방향을 추출합니다.

  * `linear.x`: 전진/후진
  * `linear.y`: 좌/우 회전
  * `linear.z`: 속도

```python
if go_back > 0:
    self.robot.move_run_mode(self.robot.GO_FORWARD, speed)
elif go_back < 0:
    self.robot.move_run_mode(self.robot.GO_BACKWARD, speed)
elif rotate > 0:
    self.robot.move_run_mode(self.robot.TURN_LEFT, speed)
elif rotate < 0:
    self.robot.move_run_mode(self.robot.TURN_RIGHT, speed)
else:
    self.robot.move_run_mode(self.robot.STOP, 0)
```

* 방향과 속도에 따라 모터 제어 명령을 보드에 전달합니다.

---

### 7. 메인 루프 실행

```python
def main_loop(self):
    while rclpy.ok():
        rclpy.spin_once(self)
```

* 메인 루프에서 타이머와 콜백 함수들이 실행됩니다.

---

## ✅ 실행 전 확인 요약

1. 연결된 시리얼 포트 확인:

```bash
ls /dev/tty*
```

2. 포트 권한 부여:

```bash
sudo chmod 666 /dev/ttyACM0
```

또는:

```bash
sudo usermod -a -G dialout $USER
# → 로그아웃 후 재로그인 필요
```

3. 노드 실행:

```bash
ros2 run jdamr200_node jdamr200_node
```

---

필요하다면 이 내용을 `README.md` 또는 개발자 문서용 `.md` 파일로도 정리해드릴 수 있습니다. 정리해서 넣어드릴까요?


