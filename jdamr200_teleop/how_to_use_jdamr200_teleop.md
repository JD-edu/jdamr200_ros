# jdamr200 Teleop 노드 사용방법, 소스코드 설명 
## 🎮 `teleop_node.py` 개요

이 코드는 키보드 입력으로 ROS2 로봇을 원격 제어하는 **터미널 기반 텔레옵 노드**입니다.
ROS2에서 제공하는 `teleop_twist_keyboard` 기능을 기반으로 커스텀 수정한 것으로, 다음 기능이 포함되어 있습니다:

### ✅ 주요 기능

* `w`, `s`, `a`, `d` 키로 **로봇 전진/후진/좌회전/우회전 제어**
* `u`, `j` 키로 **속도 조절 (50\~100)**
* `space` 또는 `x` 키로 **정지**
* `CTRL+C`로 종료 시, 로봇 정지 후 안전 종료
* `Twist` 메시지를 `/cmd_vel` 토픽으로 발행

---

## 🏁 실행 방법

ROS2 환경을 활성화한 후 다음 명령어로 실행합니다:

```bash
source ~/jdamr200_ws/install/setup.bash
ros2 run jdamr200_teleop teleop_node
```

실행 시 아래와 같은 조작 안내가 터미널에 출력됩니다:

```
Control myagv!
---------------------------
Moving around:
        w     
   a    s    d
    
space key, k : stop
...
```

---

## 📨 `cmd_vel` 메시지 발행 구조

* 이 노드는 `geometry_msgs/msg/Twist` 메시지를 **`/cmd_vel`** 토픽으로 퍼블리시합니다.
* 이 메시지는 **로봇의 이동 명령**을 포함하고 있으며, 다음 필드를 사용합니다:

```plaintext
twist.linear.x  → 전진/후진
twist.linear.y  → 좌회전/우회전
twist.linear.z  → 속도값 (커스텀 사용)
```

---

## ⚙️ 코드 구조 설명

### 1. 노드 초기화

```python
self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
```

* `/cmd_vel` 토픽에 메시지를 발행할 퍼블리셔를 생성합니다.

### 2. 키 입력 처리 및 제어값 설정

```python
key = self.get_key()
```

* `get_key()` 함수는 사용자의 키보드 입력을 읽습니다. non-blocking 방식으로 처리됩니다.

```python
if key == 'w':
    self.x = 0.5  # 전진
elif key == 's':
    self.x = -0.5 # 후진
elif key == 'a':
    self.y = 0.5  # 좌회전
elif key == 'd':
    self.y = -0.5 # 우회전
...
elif key == 'u':
    self.speed += 10.0  # 속도 증가
```

* `x`, `y` 방향을 조정하고 `speed`는 `linear.z`에 설정되어 로봇 모터 속도 제어에 사용됩니다.

### 3. Twist 메시지 생성 및 퍼블리시

```python
twist = Twist()
twist.linear.x = self.x
twist.linear.y = self.y
twist.linear.z = self.speed
...
self.pub.publish(twist)
```

* `Twist` 메시지를 생성하여 현재 방향, 속도 정보를 포함해 `/cmd_vel`로 발행합니다.

---

## 🤖 `jdamr200_node`에서 cmd\_vel 처리 방식

`jdamr200_node`는 내부적으로 다음과 같이 `/cmd_vel`을 구독합니다:

```python
self.subscriber_cmd_vel = self.create_subscription(
    Twist,
    '/cmd_vel',
    self.cmd_vel_callback,
    10
)
```

* 메시지를 수신하면 `cmd_vel_callback()`이 호출됩니다.
* 이 콜백 함수는 수신된 방향과 속도 정보를 분석해 `Jdamr200` 클래스의 `move_run_mode()` 함수를 호출합니다.

  * 예: 전진 → `GO_FORWARD`, 회전 → `TURN_LEFT` 등
* 이후 해당 명령은 시리얼 통신으로 임베디드 보드에 전송되어 모터가 동작합니다.

---

## 🔄 메시지 흐름 요약

```plaintext
[teleop_node]  --- Twist(msg) -->  /cmd_vel  -->  [jdamr200_node]  --> serial --> Robot control
```

* 키보드 → ROS2 토픽 `/cmd_vel` → 모터 명령 변환 → 로봇 제어

---

이 설명을 `jdamr200_teleop`의 README 구성이나 문서로 변환할 수 있습니다. 필요하신가요?

