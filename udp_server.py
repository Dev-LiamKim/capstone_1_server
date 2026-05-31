# udp_server.py
import socket
import threading
import struct
import math
import time
from typing import Dict
import vpython as vp
import config as cfg # 설정 파일 임포트

# --- 1. Quaternion Class ---
class quaternion:
    def __init__(self, w, x, y, z):
        self.w, self.x, self.y, self.z = w, x, y, z

    def __mul__(self, other):
        return quaternion(
            self.w*other.w - self.x*other.x - self.y*other.y - self.z*other.z,
            self.w*other.x + self.x*other.w + self.y*other.z - self.z*other.y,
            self.w*other.y - self.x*other.z + self.y*other.w + self.z*other.x,
            self.w*other.z + self.x*other.y - self.y*other.x + self.z*other.w
        )
    
    def conjugate(self):
        return quaternion(self.w, -self.x, -self.y, -self.z)

    def normalize(self):
        norm = math.sqrt(self.w**2 + self.x**2 + self.y**2 + self.z**2)
        if norm == 0: return quaternion(1,0,0,0)
        return quaternion(self.w/norm, self.x/norm, self.y/norm, self.z/norm)

    def inverse(self):
        return self.conjugate().normalize()

    def rotate(self, v): 
        if hasattr(v, 'x'): vx, vy, vz = v.x, v.y, v.z
        else: vx, vy, vz = v
        v_quat = quaternion(0, vx, vy, vz)
        res = self * v_quat * self.conjugate()
        return vp.vector(res.x, res.y, res.z)

    @staticmethod
    def slerp(qa, qb, t):
        cos_half = qa.w*qb.w + qa.x*qb.x + qa.y*qb.y + qa.z*qb.z
        if abs(cos_half) >= 1.0: return qa
        if cos_half < 0:
            qa = quaternion(-qa.w, -qa.x, -qa.y, -qa.z)
            cos_half = -cos_half
        sin_half = math.sqrt(1.0 - cos_half**2)
        if abs(sin_half) < 0.001:
            return quaternion(qa.w*0.5+qb.w*0.5, qa.x*0.5+qb.x*0.5, qa.y*0.5+qb.y*0.5, qa.z*0.5+qb.z*0.5)
        half_theta = math.acos(cos_half)
        ra = math.sin((1-t)*half_theta) / sin_half
        rb = math.sin(t*half_theta) / sin_half
        return quaternion(qa.w*ra+qb.w*rb, qa.x*ra+qb.x*rb, qa.y*ra+qb.y*rb, qa.z*ra+qb.z*rb)

# --- 2. UDP Server Class ---
class UdpDataServer:
    def __init__(self):
        self.current_quats = {i: quaternion(1,0,0,0) for i in cfg.NODE_IDS}
        self.target_quats = {i: quaternion(1,0,0,0) for i in cfg.NODE_IDS}
        self.offset_quats = {i: quaternion(1,0,0,0) for i in cfg.NODE_IDS}
        self.recv_counts = {i: 0 for i in cfg.NODE_IDS}
        
        self.data_lock = threading.Lock()
        self.is_running = True
        self.sock = None
        self.thread = None
        self.packet_size = struct.calcsize(cfg.PACKET_FMT)

    def start(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.bind((cfg.UDP_IP, cfg.UDP_PORT))
            self.sock.setblocking(False) 
            self.thread = threading.Thread(target=self._udp_task, daemon=True)
            self.thread.start()
            print(f"UDP Listener started on port {cfg.UDP_PORT}")
        except Exception as e:
            print(f"Failed to start UDP server: {e}")
            self.is_running = False

    def stop(self):
        self.is_running = False
        if self.sock: self.sock.close()
        if self.thread: self.thread.join(1)
        print("UDP Listener stopped.")

    def _udp_task(self):
        while self.is_running:
            try:
                data, _ = self.sock.recvfrom(1024)
                if len(data) == self.packet_size:
                    idx, w, x, y, z = struct.unpack(cfg.PACKET_FMT, data)
                    with self.data_lock:
                        if idx in self.target_quats:
                            self.target_quats[idx] = quaternion(w, x, y, z).normalize()
                            self.recv_counts[idx] += 1
                            if self.recv_counts[idx] % 20 == 0:
                                print(f"[RECV] ID:{idx} | Q({w:.2f}, {x:.2f}, {y:.2f}, {z:.2f})")
            except BlockingIOError: pass
            except Exception: pass
            time.sleep(0.0001)

    def update_current_quats(self, slerp_factor=0.1):
        with self.data_lock:
            for i in cfg.NODE_IDS:
                target = self.offset_quats[i] * self.target_quats[i]
                self.current_quats[i] = quaternion.slerp(self.current_quats[i], target, slerp_factor)
        return self.current_quats # 참조 반환

    def calibrate(self):
        with self.data_lock:
            for i in self.target_quats:
                self.offset_quats[i] = self.target_quats[i].inverse()
        print("\n[Calibration] T-Pose Offset Updated!")

    def get_quat(self, node_id):
        with self.data_lock: return self.current_quats[node_id]

    def get_count(self, node_id):
        with self.data_lock: return self.recv_counts[node_id]