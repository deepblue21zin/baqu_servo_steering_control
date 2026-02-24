import socket
import struct
import time

# ===== 사용자 설정 =====
STM32_IP = "10.177.21.4"   # 보드 IP
PORT = 5000                # 수신 포트
INTERVAL = 0.1             # 전송 주기 (초)

# 현재 코드 필터 기준:
# ASMS sender 마지막 옥텟=5, PC sender 마지막 옥텟=1
# 즉, 이 스크립트는 PC가 10.177.21.1일 때 정상 동작(PC 패킷 인식)합니다.

def send_asms_mode(sock, mode, joy_x=0, joy_y=0, count=10):
    """
    ASMS 패킷 5B:
      [0] mode(uint8)
      [1..2] joy_x(int16 LE)
      [3..4] joy_y(int16 LE)
    """
    pkt = struct.pack("<Bhh", mode, joy_x, joy_y)
    for i in range(count):
        sock.sendto(pkt, (STM32_IP, PORT))
        print(f"[ASMS] {i+1}/{count} sent {len(pkt)}B mode={mode} joy_x={joy_x} joy_y={joy_y}")
        time.sleep(INTERVAL)

def send_pc_cmd(sock, steer, speed, misc=0, count=10):
    """
    PC 패킷 9B:
      [0..3] steer(int32 LE)
      [4..7] speed(uint32 LE)
      [8] misc(uint8)
    """
    pkt = struct.pack("<iIB", steer, speed, misc)
    for i in range(count):
        sock.sendto(pkt, (STM32_IP, PORT))
        print(f"[PC]   {i+1}/{count} sent {len(pkt)}B steer={steer} speed={speed} misc={misc}")
        time.sleep(INTERVAL)

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print("=== UDP TEST START ===")
    print(f"Target: {STM32_IP}:{PORT}")

    # 1) AUTO 모드 진입 (ASMS mode=1)
    send_asms_mode(sock, mode=1, joy_x=0, joy_y=0, count=10)

    # 2) PC 조향 명령 전송 (AUTO 모드에서 처리됨)
    send_pc_cmd(sock, steer=30, speed=500, misc=0, count=20)
    send_pc_cmd(sock, steer=-30, speed=500, misc=0, count=20)
    send_pc_cmd(sock, steer=0, speed=0, misc=0, count=10)

    print("=== UDP TEST END ===")

if __name__ == "__main__":
    main()
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("10.177.21.1", 0))   # PC 실제 IP로 바인드
