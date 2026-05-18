#!/usr/bin/env python3
"""
relay_pi.py - Runs on the Raspberry Pi 5
Bridges Serial to TCP AND handles local PS5 controller input.
"""

import argparse
import serial
import socket
import threading
import time
import sys
import os

# Try to import pygame for local controller support
try:
    import pygame
    HAS_PYGAME = True
except ImportError:
    HAS_PYGAME = False

# ─── Controller Mapping (Matches original script) ────────────────────────────
DEAD = 0.12
AX_LX=0; AX_LY=1; AX_RX=3; AX_RY=4

def _ax(joy, n):
    if not joy: return 0.0
    v = joy.get_axis(n) if joy.get_numaxes() > n else 0.0
    # The fix for the extreme values bug
    return 0.0 if abs(v) < DEAD else v

def _b(joy, n):
    return bool(joy.get_button(n)) if joy and joy.get_numbuttons() > n else False

def _i(v):
    return int(max(-100, min(100, v * 100)))

def pkt_joy(joy):
    BTN_CROSS=0; BTN_CIRCLE=1; BTN_SQUARE=2; BTN_TRIANGLE=3
    BTN_L1=4;    BTN_R1=5;     BTN_L2=6;    BTN_R2=7
    BTN_L3=11;   BTN_R3=12

    lsx, lsy = _i(_ax(joy, AX_LX)), _i(_ax(joy, AX_LY))
    rsx, rsy = _i(_ax(joy, AX_RX)), _i(_ax(joy, AX_RY))
    hat = joy.get_hat(0) if joy.get_numhats() > 0 else (0, 0)
    face = (_b(joy,BTN_SQUARE)<<0)|(_b(joy,BTN_CROSS)<<1)|(_b(joy,BTN_CIRCLE)<<2)|(_b(joy,BTN_TRIANGLE)<<3)
    trig = (_b(joy,BTN_L3)<<0)|(_b(joy,BTN_L2)<<1)|(_b(joy,BTN_L1)<<2)|(_b(joy,BTN_R3)<<3)|(_b(joy,BTN_R2)<<4)|(_b(joy,BTN_R1)<<5)
    return f"/CONTROLL/{rsx},{rsy},{lsx},{lsy},{hat[0]},{hat[1]},{face},0,{trig}\n"

# ─── Threading Logic ─────────────────────────────────────────────────────────

def serial_to_socket(ser, client_sock):
    try:
        while True:
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                client_sock.sendall(data)
            else:
                time.sleep(0.001)
    except:
        pass

def socket_to_serial(client_sock, ser):
    try:
        while True:
            data = client_sock.recv(1024)
            if not data:
                break
            ser.write(data)
    except:
        pass

def local_controller_thread(ser, client_sock_ref):
    if not HAS_PYGAME:
        return
    
    pygame.init()
    pygame.joystick.init()
    
    joy = None
    if pygame.joystick.get_count() > 0:
        joy = pygame.joystick.Joystick(0)
        joy.init()
        print(f"Local controller detected: {joy.get_name()}")
    else:
        print("No local controller found on Pi.")
        return

    hz = 50
    interval = 1.0 / hz
    
    try:
        while True:
            start_time = time.time()
            pygame.event.pump()
            
            pkt = pkt_joy(joy)
            ser.write(pkt.encode("ascii"))
            
            # Also send to laptop if connected
            sock = client_sock_ref[0]
            if sock:
                try:
                    sock.sendall(pkt.encode("ascii"))
                except:
                    client_sock_ref[0] = None
            
            elapsed = time.time() - start_time
            time.sleep(max(0, interval - elapsed))
    except Exception as e:
        print(f"Local controller error: {e}")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baud", default=921600, type=int)
    parser.add_argument("--listen", default="0.0.0.0")
    parser.add_argument("--netport", default=5000, type=int)
    parser.add_argument("--local", action="store_true", help="Enable local controller on Pi")
    args = parser.parse_args()

    print(f"Starting Relay on Pi...")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except Exception as e:
        print(f"Serial error: {e}")
        sys.exit(1)

    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_sock.bind((args.listen, args.netport))
    server_sock.listen(1)

    client_sock_ref = [None]

    if args.local:
        threading.Thread(target=local_controller_thread, args=(ser, client_sock_ref), daemon=True).start()

    try:
        while True:
            print("\nWaiting for connection...")
            client_sock, addr = server_sock.accept()
            print(f"Connected by {addr}")
            client_sock_ref[0] = client_sock

            t1 = threading.Thread(target=serial_to_socket, args=(ser, client_sock), daemon=True)
            t2 = threading.Thread(target=socket_to_serial, args=(client_sock, ser), daemon=True)
            
            t1.start()
            t2.start()

            while t1.is_alive() and t2.is_alive():
                time.sleep(1)
            
            print("Connection lost.")
            client_sock.close()
            client_sock_ref[0] = None
    except KeyboardInterrupt:
        print("\nExit.")
    finally:
        ser.close()
        server_sock.close()

if __name__ == "__main__":
    main()
