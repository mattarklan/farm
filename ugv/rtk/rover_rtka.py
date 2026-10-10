#!/usr/bin/env python3
import os
import sys
import time
import serial

# --- UDP bus (replaces the old rover_pos.dat file) ---
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from common.protocol import Publisher, ProtocolError

# --- Configuration ---
LORA_PORT = "COM3"
LORA_BAUD = 9600

NMEA_PORT = "COM12"
NMEA_BAUD = 115200

_pub = Publisher()

def publish_shared_position(lat, lon, fix_type, sats, hdop, corr_age):
    """Publish the current fix on the UDP bus (port 5007), in the format defined in common/protocol.py."""
    try:
        _pub.send("rtk", lat=lat, lon=lon, fix=fix_type, sats=sats, hdop=hdop, corr_age=corr_age)
    except ProtocolError as e:
        print(f"[PROTOCOL ERROR] not publishing: {e}")

def open_serial_safe(port, baud):
    """Attempts to connect to a serial port continuously until successful."""
    while True:
        try:
            ser = serial.Serial(port, baud, timeout=1)
            print(f"[SERIAL] Connected to {port} @ {baud}")
            return ser
        except serial.SerialException:
            print(f"[SERIAL ERROR] Could not open {port}. Retrying in 2s...")
            time.sleep(2)

def main():
    print(f"LoRa       : {LORA_PORT} @ {LORA_BAUD}")
    print(f"NMEA/RTCM  : {NMEA_PORT} @ {NMEA_BAUD}")
    print("Publishing  : UDP bus, rtk on port 5007 (see common/PROTOCOL.md)")
    print("Ctrl+C to stop.\n")

    lora_ser = open_serial_safe(LORA_PORT, LORA_BAUD)
    gnss_ser = open_serial_safe(NMEA_PORT, NMEA_BAUD)

    rtcm_bytes_total = 0
    rtcm_count = 0

    while True:
        try:
            # 1. Forward incoming LoRa RTCM packets to GNSS module
            if lora_ser.in_waiting > 0:
                rtcm_data = lora_ser.read(lora_ser.in_waiting)
                gnss_ser.write(rtcm_data)
                rtcm_bytes_total += len(rtcm_data)
                rtcm_count += 1

            # 2. Read NMEA stdout sentences from GNSS module
            if gnss_ser.in_waiting > 0:
                line = gnss_ser.readline().decode('utf-8', errors='ignore').strip()

                if line.startswith(('$GNGGA', '$GPGGA')):
                    parts = line.split(',')
                    # Safely check length before accessing index 6
                    if len(parts) > 6 and parts[2] and parts[4]:
                        try:
                            raw_lat, lat_dir = float(parts[2]), parts[3]
                            raw_lon, lon_dir = float(parts[4]), parts[5]

                            lat = (int(raw_lat / 100)) + ((raw_lat % 100) / 60.0)
                            if lat_dir == 'S': lat = -lat

                            lon = (int(raw_lon / 100)) + ((raw_lon % 100) / 60.0)
                            if lon_dir == 'W': lon = -lon

                            fix_code = parts[6] if len(parts) > 6 else "0"
                            sats = int(parts[7]) if len(parts) > 7 and parts[7] else 0
                            hdop = float(parts[8]) if len(parts) > 8 and parts[8] else 99.0
                            corr_age = float(parts[13]) if len(parts) > 13 and parts[13] else 0.0

                            fix_map = {"0": "No Fix", "1": "3D Fix", "2": "DGNSS", "4": "Fixed RTK", "5": "Float RTK"}
                            fix_type = fix_map.get(fix_code, "Unknown")

                            print(f"[GNSS] {fix_type:<10} | sats={sats:02d} hdop={hdop:.2f} corr_age={corr_age:.1f}s | "
                                  f"lat={lat:.7f}, lon={lon:.7f} | [LoRa] Packets={rtcm_count} ({rtcm_bytes_total}B)")

                            publish_shared_position(lat, lon, fix_type, sats, hdop, corr_age)
                        except (ValueError, IndexError):
                            pass

            time.sleep(0.01)

        except (serial.SerialException, OSError) as e:
            print(f"\n[WARNING] Serial connection lost ({e}). Attempting reconnect...")
            try:
                lora_ser.close()
                gnss_ser.close()
            except Exception:
                pass
            time.sleep(1)
            lora_ser = open_serial_safe(LORA_PORT, LORA_BAUD)
            gnss_ser = open_serial_safe(NMEA_PORT, NMEA_BAUD)

        except KeyboardInterrupt:
            print("\nExiting rover_rtk...")
            break

    lora_ser.close()
    gnss_ser.close()

if __name__ == "__main__":
    main()