# LAICA_enclc

DYLY-108 로드셀과 EN25-Absolute 엔코더 데이터를 Arduino Mega로 수집하여 ROS 토픽으로 발행하는 패키지입니다.

---

## 시스템 구조

```
DYLY-108 로드셀
    │ (AIN0/AIN1 차동 입력)
    ▼
ADS1256 ADC 모듈 ─── SPI ───┐
                              ├──► Arduino Mega ──── USB ────► ROS (PC / Raspberry Pi)
EN25-Absolute 엔코더 ─ TTL ──┘
    (Serial1, Pin 19)
```

Arduino가 두 센서를 읽어 USB 시리얼로 내보내면, ROS 노드가 이를 파싱해 두 개의 토픽으로 발행합니다.

---

## 하드웨어 구성

### 사용 부품

| 부품 | 모델 | 역할 |
|------|------|------|
| MCU | Arduino Mega 2560 | 센서 데이터 수집 및 USB 전송 |
| ADC | ADS1256 모듈 (TI ADS1256IDB) | 로드셀 24비트 A/D 변환 |
| 로드셀 | DYLY-108 | 힘 측정 |
| 엔코더 | EN25-Absolute | 절대 각도 측정 |

### 배선

**① EN25-Absolute → Arduino Mega**

| 엔코더 핀 | Arduino Mega | 비고 |
|-----------|-------------|------|
| 🔴 Red (5V+) | 5V | 전원 |
| 🟢 Green (GND) | GND | 접지 |
| 🟡 Yellow (TTL TX) | **19번 (RX1)** | 시리얼 수신 |

**② ADS1256 모듈 → Arduino Mega**

| ADS1256 핀 | Arduino Mega | 비고 |
|------------|-------------|------|
| 5V | 5V | 전원 |
| GND | GND | 접지 |
| SCLK | **52번** | SPI Clock |
| DIN | **51번** | SPI MOSI |
| DOUT | **50번** | SPI MISO |
| CS | **53번** | Chip Select |
| DRDY | **48번** | 데이터 준비 신호 (Active Low) |
| PDWN | **49번** | Power Down / Reset (HIGH = 동작) |

**③ DYLY-108 로드셀 → ADS1256**

| 로드셀 색 | ADS1256 | 비고 |
|-----------|---------|------|
| 🔴 Red | 5V | 가진 전압(+) |
| ⚫ Black | GND | 가진 전압(-) |
| 🟢 Green | **AIN0** | 신호(+) |
| ⚪ White | **AIN1** | 신호(-) |

> ADS1256는 AIN0(+) − AIN1(−) 차동 입력으로 로드셀 신호를 읽습니다.

---

## 파일 구조

```
LAICA_enclc/
├── arduino/
│   └── ads1256_encoder/
│       └── ads1256_encoder.ino  # Arduino Mega 스케치
├── msg/
│   ├── EncoderData.msg          # ROS 메시지: 엔코더
│   └── LoadCellData.msg         # ROS 메시지: 로드셀
├── src/
│   └── arduino_sensor_pub.cpp   # ROS C++ 노드 (USB 시리얼 → 토픽)
├── config/
│   └── params.yaml              # 파라미터 설정
└── launch/
    └── sensors.launch           # 실행 런치 파일
```

---

## 설치 및 실행

### 1단계: Arduino 스케치 업로드

1. Arduino IDE에서 `arduino/ads1256_encoder/ads1256_encoder.ino` 열기
2. **보드**: `Arduino Mega or Mega 2560` 선택
3. **포트**: 연결된 COM 포트 선택
4. 업로드 후 시리얼 모니터(115200 baud)에서 아래 형식의 출력 확인:

```
# enc_lc ready
$SENSORS,-12543,2.3456,152.3,11.644,110
$SENSORS,-12540,2.3453,152.5,11.645,110
...
```

> 출력이 보이면 Arduino 쪽은 정상입니다.

Serial Monitor에서 사람이 읽기 쉽게 확인하려면 sketch 상단에서:

```cpp
#define OUTPUT_MODE MONITOR_MODE
```

로 바꿔 업로드하세요. ROS 2 노드와 연결할 때는 다시:

```cpp
#define OUTPUT_MODE ROS_MODE
```

로 두어야 합니다.

### 2단계: ROS 패키지 빌드

```bash
cp -r LAICA_enclc ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --packages-select enc_lc
source install/setup.bash
```

### 3단계: 포트 확인 및 설정

Linux에서 Arduino Mega는 보통 `/dev/ttyACM0`으로 잡힙니다.

```bash
ls /dev/ttyACM*   # 포트 확인
```

`config/params.yaml`에서 포트 수정:

```yaml
port: /dev/ttyACM0   # 확인한 포트로 변경
```

### 4단계: 실행

```bash
ros2 launch enc_lc sensors.launch

# 또는 포트를 직접 지정
ros2 launch enc_lc sensors.launch port:=/dev/ttyACM1
```

### 5단계: 데이터 확인

```bash
ros2 topic echo /load_cell/data
ros2 topic echo /encoder/data
```

---

## ROS 토픽

| 토픽 | 메시지 타입 | 발행 주기 |
|------|------------|---------|
| `/load_cell/data` | `enc_lc/LoadCellData` | 100 Hz |
| `/encoder/data` | `enc_lc/EncoderData` | 100 Hz |

### LoadCellData.msg

```
std_msgs/Header header
int32   raw_count    # 24비트 ADC 원시값 (AIN0 - AIN1 차동)
float64 voltage_mv   # 차동 전압 [mV]
float64 force_n      # 캘리브레이션된 힘 [N]
```

### EncoderData.msg

```
std_msgs/Header header
float64 angle_deg    # 절대 각도 [0.0 ~ 359.9] degrees
float64 rev          # 누적 회전수 (전원 OFF시 초기화)
int32   rpm          # 현재 회전 속도 [RPM]
```

---

## 로드셀 캘리브레이션

`raw_count`는 ADC 원시값 그대로 발행됩니다. 모델 입력으로 raw loadcell 값이 필요하면
`raw_count`를 사용하세요.

`force_n`은 로드셀 전압값 `voltage_mv`에 선형회귀식을 적용해 계산합니다.

**캘리브레이션 공식:**
```
force_n = force_gradient_n_per_mv × voltage_mv + force_bias_n
```

현재 `config/params.yaml`의 값:

```yaml
force_gradient_n_per_mv: 283.07
force_bias_n: 0.05122
```

참고용으로 보존한 다른 pair:

```yaml
# force_gradient_n_per_mv: 1.318e-03
# force_bias_n: -14.5
```

---

## ADS1256 PGA 설정

스케치 상단의 `PGA_SEL`을 로드셀 스펙에 맞게 조정하세요.

```cpp
#define PGA_SEL 64   // 기본값: PGA = 64 (범위 ±39.1 mV)
```

| PGA 값 | 차동 입력 범위 | 적합한 로드셀 |
|--------|-------------|-------------|
| 1 | ±2500 mV | - |
| 8 | ±312.5 mV | - |
| 32 | ±78.1 mV | 고출력 로드셀 |
| **64** | **±39.1 mV** | **DYLY-108 (권장)** |

> DYLY-108 기준: 감도 ~2 mV/V, 가진 5V → 풀스케일 출력 ≈ 10 mV → PGA=64 적합
>
> PGA 변경 후에는 반드시 스케치를 다시 업로드하세요.

---

## 시리얼 데이터 포맷 (Arduino → ROS)

Arduino Mega가 USB로 전송하는 포맷입니다.

```
$SENSORS,<lc_raw>,<lc_mv>,<enc_deg>,<enc_rev>,<enc_rpm>\r\n
```

| 필드 | 설명 | 예시 |
|------|------|------|
| `lc_raw` | 24비트 부호 있는 ADC 원시값 | `-12543` |
| `lc_mv` | 차동 전압 [mV], 소수점 4자리 | `2.3456` |
| `enc_deg` | 절대 각도 [°], 소수점 1자리 | `152.3` |
| `enc_rev` | 누적 회전수, 소수점 3자리 | `11.644` |
| `enc_rpm` | 회전 속도 [RPM] | `110` |

---

## 문제 해결

**시리얼 모니터에 출력이 없을 때**
- DRDY 핀(48번) 연결 확인
- PDWN 핀(49번)이 5V에 연결되어 있는지 확인
- SPI 배선(50/51/52/53번) 재확인

**lc_mv 값이 포화(±39mV 근처)될 때**
- `PGA_SEL`을 `32` 또는 `16`으로 낮추고 재업로드

**엔코더 데이터가 0으로 고정될 때**
- Yellow 선(TTL TX)이 Mega 19번 핀(RX1)에 연결되어 있는지 확인
- 엔코더 전원(5V, GND) 확인

**ROS에서 포트를 못 찾을 때**
- `ls /dev/ttyACM*` 또는 `ls /dev/ttyUSB*` 로 포트명 확인
- `sudo chmod 666 /dev/ttyACM0` 으로 권한 부여 (또는 `dialout` 그룹 추가)
