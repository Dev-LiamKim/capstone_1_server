**README.md**

```markdown
# 6-Node Modular Arm Tracker

VPython을 이용한 6노드 상지(Arm) 모션 트래킹 시뮬레이션 시스템입니다. UDP 통신을 통해 센서 데이터를 수신하고 순기구학(Forward Kinematics)을 기반으로 3D 스켈레톤을 실시간 시각화합니다.

## 프로젝트 폴더 구조

```text
skeleton/
├── .gitignore          # __pycache__ 및 불필요 파일 제외 설정
├── config.py           # 시스템 상수, 신체 치수 및 공통 설정
├── udp_server.py       # Quaternion 연산 및 스레드 기반 UDP 수신 서버
├── visualizer.py       # VPython 기반 3D 스켈레톤 생성 및 FK 애니메이션
└── main.py             # 시스템 초기화 및 메인 루프 실행 파일 (진입점)

```

## 모듈별 상세 기능

### 1. `config.py` (설정 관리)

* **통신 설정**: UDP IP(`0.0.0.0`), Port(`4210`), 패킷 포맷(`<iffff`) 정의
* **센서 ID 매핑**: 좌/우 상완, 하완, 손에 해당하는 6개 노드 ID 지정 (0~5)
* **신체 치수**: 가슴, 위팔, 아래팔, 손의 길이 및 크기 설정
* **시뮬레이션 파라미터**: 메인 루프 주기(`LOOP_RATE = 60Hz`) 및 SLERP 보간 계수 설정

### 2. `udp_server.py` (데이터 수신 및 연산)

* **`quaternion` 클래스**: 쿼터니언 곱셈, 켤레, 정규화, 역산 및 VPython 벡터 회전(`rotate`) 기능 제공. 구면 선형 보간(`slerp`) 지원.
* **`UdpDataServer` 클래스**: 백그라운드 스레드에서 비동기적으로 UDP 패킷을 수신하여 target 데이터 갱신. 데이터 접근 시 `Lock`을 활용한 스레드 안전성 확보. `calibrate` 기능을 통해 T-Pose 오프셋 계산.

### 3. `visualizer.py` (3D 시각화)

* **`SkeletonVisualizer` 클래스**: VPython scene 초기화 및 3D 실린더, 구, 박스 객체로 상지 스켈레톤 모델링.
* **순기구학(FK) 구현**: 월드 좌표 기준의 센서 절대 방향 데이터(`q_world`)를 계층 구조(상완 -> 하완 -> 손)에 따라 상대 변환 및 결합하여 각 관절 위치와 뼈대 축(`axis`)을 실시간 업데이트. 데이터 수신 상태에 따른 실시간 노드 색상 변경 로직 포함.

### 4. `main.py` (실행 진입점)

* `UdpDataServer` 및 `SkeletonVisualizer` 인스턴스 생성.
* 키보드 'c' 입력 이벤트 바인딩을 통한 실시간 T-Pose 캘리브레이션 트리거.
* 설정된 주기(`LOOP_RATE`)에 맞추어 보간 연산(`update_current_quats`)과 시각화 갱신(`update_pose`)을 수행하는 메인 루프 구동.

## 실행 방법

1. **의존성 라이브러리 설치**
```bash
pip install vpython

```


2. **프로그램 실행**
```bash
python main.py

```


3. **캘리브레이션**
* 시스템이 시작되면 착용자가 T-Pose를 취한 상태에서 키보드 `c`를 눌러 오프셋을 보정합니다.



```

```
