#!/usr/bin/env python3
import socket, struct, time, threading
from uart_cp import UcpCtlCmd, UCP_MOTOR_CTL

# ===========================================================
# ---- CRC16 identical to firmware --------------------------
# ===========================================================
def crc16(data: bytes) -> int:
    crc_hi = 0xFF
    crc_lo = 0xFF
    crc_hi_table = [
        0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
        0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
    ] * 16  # shortened; same pattern repeated
    crc_lo_table = [
        0x00,0xC0,0xC1,0x01,0xC3,0x03,0x02,0xC2,
        0xC6,0x06,0x07,0xC7,0x05,0xC5,0xC4,0x04,
    ] * 16
    for b in data:
        index = crc_lo ^ b
        crc_lo = crc_hi ^ crc_hi_table[index]
        crc_hi = crc_lo_table[index]
    return (crc_hi << 8) | crc_lo

# ===========================================================
# ---- Frame extraction and decoding ------------------------
# ===========================================================
HEADER = b"\xFD\xFF"

def extract_frames(buf):
    frames = []
    i = 0
    while i < len(buf) - 4:
        if buf[i:i+2] == HEADER:
            length = buf[i+2] | (buf[i+3] << 8)
            total = 2 + 2 + length + 2
            if i + total <= len(buf):
                frames.append(buf[i:i+total])
                i += total
            else:
                break
        else:
            i += 1
    return frames, buf[i:]

def decode_ucp_0x05(frame):
    """Decode the 0x05 telemetry packet (state report)."""
    if len(frame) < 44:
        return None
    (
        h1,h2,len_lo,len_hi,pkt_id,index,battery,
        rpm0,rpm1,rpm2,rpm3,
        acc_x,acc_y,acc_z,
        gyro_x,gyro_y,gyro_z,
        mag_x,mag_y,mag_z,
        heading,power,volt_tenth,curr_centi,version,crc
    ) = struct.unpack("<2B2BHBBhhhhhhhhhhhhhhBBHHH", frame[:44])
    return {
        "pkt_id": pkt_id,
        "index": index,
        "battery_mV": battery,
        "rpm": [rpm0,rpm1,rpm2,rpm3],
        "acc": [acc_x,acc_y,acc_z],
        "gyro": [gyro_x,gyro_y,gyro_z],
        "mag": [mag_x,mag_y,mag_z],
        "heading_deg": heading/100.0,
        "voltage_V": volt_tenth/10.0,
        "current_A": curr_centi/100.0,
    }

# ===========================================================
# ---- Sender helpers (from your move.py) -------------------
# ===========================================================
def send_ctl_cmd(sock, speed, angular):
    cmd = UcpCtlCmd()
    cmd.hd.len = len(bytes(cmd))
    cmd.hd.id = UCP_MOTOR_CTL
    cmd.hd.index = 0
    cmd.speed = speed
    cmd.angular = angular
    head = 0xfffd
    payload = bytes(cmd)
    buf = struct.pack("<H", head) + payload
    crc = crc16(buf)
    buf += struct.pack("<H", crc)
    sock.sendall(buf)

def robot_move(sock, duration=3.0, speed=60, angular=0):
    print(f"[MOVE] speed={speed}, angular={angular}")
    start = time.time()
    while time.time() - start < duration:
        send_ctl_cmd(sock, speed, angular)
        time.sleep(0.1)
    send_ctl_cmd(sock, 0, 0)
    print("[MOVE] stop")

# ===========================================================
# ---- Reader thread ----------------------------------------
# ===========================================================
def reader_loop(sock, stop_event):
    buf = b""
    print("[READER] Telemetry thread started")
    while not stop_event.is_set():
        try:
            data = sock.recv(1024)
            if not data:
                print("[READER] Socket closed by rover")
                break
            buf += data
            frames, buf = extract_frames(buf)
            for f in frames:
                pkt_id = f[4]
                if pkt_id == 0x05:
                    decoded = decode_ucp_0x05(f)
                    if decoded:
                        print(f"[TELE] Batt={decoded['battery_mV']}mV "
                              f"RPM={decoded['rpm']} "
                              f"Head={decoded['heading_deg']:.1f}° "
                              f"V={decoded['voltage_V']:.1f}V I={decoded['current_A']:.2f}A")
        except socket.timeout:
            continue
        except Exception as e:
            print("[READER] Error:", e)
            break
    print("[READER] Exit")

# ===========================================================
# ---- Main -------------------------------------------------
# ===========================================================
if __name__ == "__main__":
    HOST = "192.168.11.1"
    PORT = 8888

    sock = socket.create_connection((HOST, PORT))
    # sock.settimeout(0.5)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    # stop_event = threading.Event()
    # t = threading.Thread(target=reader_loop, args=(sock, stop_event), daemon=True)
    # t.start()

    try:
        input("Press Enter to move forward...")
        robot_move(sock, duration=3.0, speed=100, angular=0)

        input("Press Enter to move backward...")
        robot_move(sock, duration=3.0, speed=-100, angular=0)

        input("Press Enter to turn right...")
        robot_move(sock, duration=3.0, speed=60, angular=360)

    finally:
        # stop_event.set()
        # t.join(timeout=2)
        sock.close()
        print("[MAIN] Connection closed")
