#!/usr/bin/env python3
import socket
import struct
import time

HOST = "192.168.11.1"   # rover IP
PORT = 8888
MAX_LEN = 512           # sanity limit for packet length

HEADER = b"\xFD\xFF"
def extract_frames(buffer):
    """Finds and extracts complete frames from a running byte buffer."""
    frames = []
    i = 0
    while i < len(buffer) - 4:
        # look for header
        if buffer[i:i+2] == HEADER:
            if i + 4 > len(buffer):
                break
            length = buffer[i+2] | (buffer[i+3] << 8)
            total_len = 2 + 2 + length + 2  # header + len + payload + CRC16
            if i + total_len <= len(buffer):
                frame = buffer[i:i+total_len]
                frames.append(frame)
                i += total_len
            else:
                break  # wait for more data
        else:
            i += 1
    # Keep the leftover incomplete data
    remaining = buffer[i:]
    return frames, remaining

def parse_frame(frame):
    """Decode one binary frame into human-readable numbers."""
    if len(frame) < 6:
        return None

    length = struct.unpack("<H", frame[2:4])[0]
    cmd = frame[4]
    data = frame[5:-2]
    crc = frame[-2:]

    # unpack as signed 16-bit ints (ignore leftover byte)
    n = len(data) // 2
    remainder = len(data) % 2
    values = struct.unpack("<" + "h" * n, data[:n*2])

    return {
        "len": length,
        "cmd": cmd,
        "crc": crc.hex(),
        "values": values,
        "remainder": data[n*2:].hex() if remainder else "",
    }

def decode_ucp_0x05(data: bytes):
    if len(data) < 44:
        raise ValueError("Incomplete frame")

    # Unpack exactly as in uart_report_state()
    (
        head1, head2,
        len_lo, len_hi,
        pkt_id, index,
        battery,
        rpm0, rpm1, rpm2, rpm3,
        acc_x, acc_y, acc_z,
        gyro_x, gyro_y, gyro_z,
        mag_x, mag_y, mag_z,
        heading,
        power, voltage_tenth,
        current_centi,  # note: little-endian 16-bit
        version, crc
    ) = struct.unpack("<2B2BHBBhhhhhhhhhhhhhhBBHHH", data)

    return {
        "packet_id": pkt_id,
        "index": index,
        "battery_mV": battery,
        "rpm": [rpm0, rpm1, rpm2, rpm3],
        "acc": [acc_x, acc_y, acc_z],
        "gyro": [gyro_x, gyro_y, gyro_z],
        "mag": [mag_x, mag_y, mag_z],
        "heading": heading / 100.0,
        "power": power,
        "voltage_V": voltage_tenth / 10.0,
        "current_A": current_centi / 100.0,
        "version": version,
    }

def main():
    print(f"[Reader] Connecting to {HOST}:{PORT} ...")
    s = socket.create_connection((HOST, PORT))
    s.settimeout(1.0)

    buf = b""
    frame_count = 0
    start_time = time.time()

    print("[Reader] Connected. Waiting for frames...")
    while True:
        try:
            chunk = s.recv(1024)
            if not chunk:
                print("[Reader] Connection closed.")
                break
            buf += chunk

            frames, buf = extract_frames(buf)
            for f in frames:
                parsed = parse_frame(f)
                if not parsed:
                    continue
                frame_count += 1

                # pretty-print
                cmd = parsed["cmd"]
                length = parsed["len"]
                vals = parsed["values"]
                print(f"Cmd={cmd:02X}  Len={length:<5}  "
                      f"FirstVals={vals[:6]}  ...  "
                      f"CRC={parsed['crc']}")

                # optional: detect anomalies
                if length > 100:
                    print(f"⚠️  Unusually long frame ({length} bytes)")

        except socket.timeout:
            continue
        except KeyboardInterrupt:
            print("\n[Reader] Interrupted. Received", frame_count, "frames in",
                  round(time.time() - start_time, 1), "s.")
            break

    s.close()


if __name__ == "__main__":
    main()
