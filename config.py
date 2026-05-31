# config.py
import vpython as vp

# --- 통신 설정 ---
UDP_IP = "0.0.0.0"
UDP_PORT = 4210
PACKET_FMT = '<iffff'

# --- 센서 ID 매핑 ---
NODE_MAP = {
    'L_UPPER': 0, 'L_LOWER': 1, 'L_HAND': 2,
    'R_UPPER': 3, 'R_LOWER': 4, 'R_HAND': 5
}
NODE_IDS = range(6)

# --- 신체 치수 (VPython 단위) ---
CHEST_H = 2.0
UPPER_LEN = 1.5
LOWER_LEN = 1.5
HAND_LEN = 0.4
BODY_RADIUS = 0.4

# --- 기본 벡터 (T-Pose) ---
V_UP = vp.vec(0, 1, 0)
V_LEFT = vp.vec(-1, 0, 0)
V_RIGHT = vp.vec(1, 0, 0)
V_FORWARD = vp.vec(0, 0, 1)

# --- 시뮬레이션 설정 ---
LOOP_RATE = 60      # 메인 루프 주기 (Hz)
SLERP_FACTOR = 0.1  # SLERP 보간 계수 (0.0 ~ 1.0)