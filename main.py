# main.py
import vpython as vp
from udp_server import UdpDataServer
from visualizer import SkeletonVisualizer
import config as cfg

def main(): 
    # 1. 모듈 초기화
    server = UdpDataServer()
    viz = SkeletonVisualizer()

    # 2. 캘리브레이션 이벤트 바인딩
    def on_key(evt):
        if evt.key == 'c':
            server.calibrate()
    vp.scene.bind('keydown', on_key)

    # 3. 서버 시작
    server.start()
    print("System Started. Press 'c' to Calibrate.")

    # 4. 메인 루프
    try:
        while True:
            vp.rate(cfg.LOOP_RATE)
            
            # 서버 데이터 업데이트 (SLERP)
            server.update_current_quats(cfg.SLERP_FACTOR)
            
            # 시각화 업데이트 (FK 계산)
            viz.update_pose(server)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        server.stop()
        print("System Terminated.")

if __name__ == "__main__":
    main()