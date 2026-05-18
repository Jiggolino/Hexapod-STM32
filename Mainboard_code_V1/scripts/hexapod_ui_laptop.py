#!/usr/bin/env python3
"""
hexapod_ui_laptop.py  —  Remote Laptop UI
PS5 controller -> TCP Socket -> Pi Relay -> STM32
Live 3-D visualiser and remote control.
"""

import argparse, logging, math, os, re, sys, threading, time, socket
from collections import deque
from datetime import datetime

import pygame

# ─── File logger ─────────────────────────────────────────────────────────────
_LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
os.makedirs(_LOG_DIR, exist_ok=True)

_log_path = os.path.join(_LOG_DIR, datetime.now().strftime("hexapod_ui_%Y%m%d_%H%M%S.log"))

logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s.%(msecs)03d  %(levelname)-7s  %(message)s",
    datefmt="%H:%M:%S",
    handlers=[
        logging.FileHandler(_log_path, encoding="utf-8"),
        logging.StreamHandler(sys.stdout),
    ],
)
log = logging.getLogger("hexapod")

_TX_LOG_EVERY = 50

# ─── Robot geometry ──────────────────────────────────────────────────────────
L1, L2, L3 = 66.93, 89.80, 165.56
RAD2DEG = 57.29578
DEG2RAD = 1.0 / RAD2DEG
NAMES   = ["FR", "MR", "BR", "BL", "ML", "FL"]
PIVOT_X = [ 125.722,   0.0,  -125.722, -125.722,    0.0,   125.722]
PIVOT_Y = [-113.425, -156.834, -113.425,  113.425,  156.834,  113.425]
MOUNT   = [math.atan2(PIVOT_Y[i], PIVOT_X[i]) for i in range(6)]
BODY_HEIGHT_MM = 140.0

def servo_to_theta(leg_idx, c_d, f_d, t_d):
    cs = +RAD2DEG if leg_idx < 3 else -RAD2DEG
    return ((c_d - 90.0) / cs, (f_d - 90.0) / +RAD2DEG, (t_d) / -RAD2DEG)

def fk3(leg_idx, t1, t2, t3):
    px, py = PIVOT_X[leg_idx], PIVOT_Y[leg_idx]
    ba  = MOUNT[leg_idx] + t1
    ca, sa = math.cos(ba), math.sin(ba)
    p0 = (px, py, 0.0)
    p1 = (px + L1 * ca, py + L1 * sa, 0.0)
    p2 = (p1[0] + L2 * math.cos(t2) * ca, p1[1] + L2 * math.cos(t2) * sa, p1[2] + L2 * math.sin(t2))
    af  = t2 + t3
    p3 = (p2[0] + L3 * math.cos(af) * ca, p2[1] + L3 * math.cos(af) * sa, p2[2] + L3 * math.sin(af))
    return p0, p1, p2, p3

# ─── Shared state ─────────────────────────────────────────────────────────────
class State:
    def __init__(self):
        self.lock      = threading.Lock()
        self.servos    = [[90.0, 90.0, 90.0] for _ in range(6)]
        self.statuses  = ['.'] * 6
        self.log       = deque(maxlen=6)
        self.connected = False
        self.err_msg   = ""

S = State()

# ─── Network reader thread ────────────────────────────────────────────────────
_SRVO = re.compile(r'([A-Z]{2})([.RL])\[C:\s*(\d+)\s+F:\s*(\d+)\s+T:\s*(\d+)\]')
_NI   = {n: i for i, n in enumerate(NAMES)}

def _parse_servo(line):
    hits = _SRVO.findall(line)
    if len(hits) < 6: return False
    with S.lock:
        for name, st, c, f, t in hits:
            i = _NI.get(name)
            if i is not None:
                S.servos[i]   = [float(c), float(f), float(t)]
                S.statuses[i] = st
    return True

def network_thread(sock):
    buf = ""
    while True:
        try:
            data = sock.recv(4096)
            if not data:
                S.connected = False
                log.warning("Connection closed by remote")
                break
            if not S.connected:
                S.connected = True
                log.info("Network connection established")
            buf += data.decode("ascii", errors="ignore")
            while "\n" in buf or "\r" in buf:
                for d in ("\r\n", "\n", "\r"):
                    if d in buf:
                        line, buf = buf.split(d, 1)
                        break
                line = line.strip()
                if not line: continue
                if _parse_servo(line):
                    pass # log.debug("RX SERVO  %s", line)
                else:
                    log.info("RX        %s", line)
                    with S.lock: S.log.append(line[:100])
        except Exception as e:
            S.connected = False
            S.err_msg = str(e)
            log.error("Network read error: %s", e)
            break

# ─── Controller Mapping ───────────────────────────────────────────────────────
DEAD = 0.12

def _ax(joy, n):
    if not joy: return 0.0
    v = joy.get_axis(n) if joy.get_numaxes() > n else 0.0
    # FIX: Extreme values near 100% (-1.0) now work correctly.
    # Triggers often rest at -1.0, but sticks should be allowed to go there.
    return 0.0 if abs(v) < DEAD else v

def _b(joy, n):
    return bool(joy.get_button(n)) if joy and joy.get_numbuttons() > n else False

def _i(v):
    return int(max(-100, min(100, v * 100)))

def pkt_joy(joy):
    # Mapping for DualSense on Ubuntu/Linux
    BTN_CROSS=0; BTN_CIRCLE=1; BTN_SQUARE=2; BTN_TRIANGLE=3
    BTN_L1=4;    BTN_R1=5;     BTN_L2=6;    BTN_R2=7
    BTN_L3=11;   BTN_R3=12
    # Standard PS5 sticks
    AX_LX=0; AX_LY=1; AX_RX=3; AX_RY=4

    lsx, lsy = _i(_ax(joy, AX_LX)), _i(_ax(joy, AX_LY))
    rsx, rsy = _i(_ax(joy, AX_RX)), _i(_ax(joy, AX_RY))
    hat = joy.get_hat(0) if joy.get_numhats() > 0 else (0, 0)
    face = (_b(joy,BTN_SQUARE)<<0)|(_b(joy,BTN_CROSS)<<1)|(_b(joy,BTN_CIRCLE)<<2)|(_b(joy,BTN_TRIANGLE)<<3)
    trig = (_b(joy,BTN_L3)<<0)|(_b(joy,BTN_L2)<<1)|(_b(joy,BTN_L1)<<2)|(_b(joy,BTN_R3)<<3)|(_b(joy,BTN_R2)<<4)|(_b(joy,BTN_R1)<<5)
    return f"/CONTROLL/{rsx},{rsy},{lsx},{lsy},{hat[0]},{hat[1]},{face},0,{trig}\n"

def pkt_keys(keys):
    lsx = (100 if keys[pygame.K_d] else 0)-(100 if keys[pygame.K_a] else 0)
    lsy = (-100 if keys[pygame.K_w] else 0)+(100 if keys[pygame.K_s] else 0)
    rsx = (100 if keys[pygame.K_e] else 0)-(100 if keys[pygame.K_q] else 0)
    trig = (4 if keys[pygame.K_l] else 0)|(32 if keys[pygame.K_r] else 0)|(2 if keys[pygame.K_LSHIFT] else 0)
    face = (1 if keys[pygame.K_1] else 0)|(8 if keys[pygame.K_2] else 0)
    return f"/CONTROLL/{rsx},0,{lsx},{lsy},0,0,{face},0,{trig}\n"

# ─── 3-D Camera & Drawing ─────────────────────────────────────────────────────
def _cam(pt, az, el):
    x, y, z = pt
    cz, sz = math.cos(az), math.sin(az)
    x, y = x*cz - y*sz, x*sz + y*cz
    ce, se = math.cos(el), math.sin(el)
    y, z = y*ce + z*se, -y*se + z*ce
    return x, y, z

def _proj(cam_pt, ox, oy, fov, cam_d):
    x, y, z = cam_pt
    depth = max(cam_d - y, 10.0)
    s  = fov / depth
    return int(ox + x * s), int(oy - z * s), depth

BG=(14,14,20); C_GRID=(30,30,42); C_BODY=(55,75,120); C_COXA=(75,75,115)
C_OK=(55,195,95); C_WARN=(215,170,45); C_ERR=(215,50,50); C_PIVOT=(170,170,170)
C_FOOT=(255,225,70); C_TEXT=(210,210,215); C_DIM=(85,85,105); C_PANEL=(20,20,28)

def draw_3d(surf, view_rect, servos, statuses, az, el, fov, cam_d):
    vx, vy, vw, vh = view_rect
    ox, oy = vx + vw // 2, vy + vh // 2
    def P(pt): return _proj(_cam(pt, az, el), ox, oy, fov, cam_d)
    segs, dots, fills = [], [], []
    gz = -BODY_HEIGHT_MM
    for gv in range(-600, 601, 100):
        for a, b in [((gv,-600,gz),(gv,600,gz)), ((-600,gv,gz),(600,gv,gz))]:
            sx1,sy1,d1 = P(a); sx2,sy2,d2 = P(b)
            segs.append(((d1+d2)/2, C_GRID, 1, (sx1,sy1), (sx2,sy2)))
    btop_pts = [(PIVOT_X[i], PIVOT_Y[i], 20.0) for i in range(6)]
    proj_top = [P(p) for p in btop_pts]
    avg_d = sum(p[2] for p in proj_top)/6
    fills.append((avg_d, C_BODY, [(p[0],p[1]) for p in proj_top]))
    for i in range(6):
        c,f,t = servos[i]
        t1,t2,t3 = servo_to_theta(i, c, f, t)
        p0,p1,p2,p3 = fk3(i, t1, t2, t3)
        col = C_ERR if statuses[i]=='R' else C_WARN if statuses[i]=='L' else C_OK
        sp0, sp1, sp2, sp3 = P(p0), P(p1), P(p2), P(p3)
        segs.append(((sp0[2]+sp1[2])/2, C_COXA, 2, sp0[:2], sp1[:2]))
        segs.append(((sp1[2]+sp2[2])/2, col, 3, sp1[:2], sp2[:2]))
        segs.append(((sp2[2]+sp3[2])/2, col, 3, sp2[:2], sp3[:2]))
        dots.append((sp0[2], C_PIVOT, sp0[:2], 4))
        dots.append((sp3[2], C_FOOT, sp3[:2], 6))
    all_draw = ([(d,'fill',c,pts) for d,c,pts in fills] +
                [(d,'seg',c,w,p1,p2) for d,c,w,p1,p2 in segs] +
                [(d,'dot',c,pos,r) for d,c,pos,r in dots])
    all_draw.sort(key=lambda x: -x[0])
    for item in all_draw:
        if item[1] == 'fill': pygame.draw.polygon(surf, item[2], item[3])
        elif item[1] == 'seg': pygame.draw.line(surf, item[2], item[4], item[5], item[3])
        else: pygame.draw.circle(surf, item[2], item[3], item[4])

def draw_info(surf, rx, ry, rw, rh, servos, statuses, last_pkt, connected, target_ip, joy, font_sm, font_md):
    pygame.draw.rect(surf, C_PANEL, (rx, ry, rw, rh))
    pygame.draw.line(surf, C_GRID, (rx, ry), (rx, ry+rh), 1)
    y = ry + 8
    def txt(s, color=C_TEXT, font=font_sm):
        nonlocal y
        surf.blit(font.render(s, True, color), (rx + 8, y))
        y += font.get_height() + 2
    txt("HEXAPOD REMOTE", C_OK if connected else C_ERR, font_md)
    txt(f"Target: {target_ip}", C_DIM)
    y += 10
    txt("SERVO ANGLES", C_DIM, font_md)
    for i in range(6):
        c, f, t = servos[i]
        txt(f"{NAMES[i]}: {int(c):3d} {int(f):3d} {int(t):3d}", _lc(statuses[i]))
    y += 10
    txt("TX PACKET:", C_DIM)
    txt(last_pkt.replace("/CONTROLL/", "")[:22], C_TEXT)

def _lc(st): return C_ERR if st=='R' else C_WARN if st=='L' else C_OK

# ─── Main ─────────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ip", default="100.122.9.49", help="Tailscale IP of the Pi")
    ap.add_argument("--port", default=5000, type=int, help="TCP port")
    ap.add_argument("--hz", default=50, type=int)
    args = ap.parse_args()

    pygame.init()
    pygame.joystick.init()
    W, H = 1280, 720
    PAN_W, VIEW_W = 220, 1280-220
    surf = pygame.display.set_mode((W, H))
    pygame.display.set_caption("Hexapod Remote Control")
    clock = pygame.time.Clock()
    font_sm = pygame.font.SysFont("monospace", 12)
    font_md = pygame.font.SysFont("monospace", 13, bold=True)

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        log.info(f"Connecting to {args.ip}:{args.port}...")
        sock.connect((args.ip, args.port))
        threading.Thread(target=network_thread, args=(sock,), daemon=True).start()
    except Exception as e:
        log.error(f"Failed to connect: {e}")
        # Continue in offline mode

    joy = pygame.joystick.Joystick(0) if pygame.joystick.get_count() > 0 else None
    if joy: joy.init()

    az, el, fov, cam_d = math.radians(30), math.radians(35), 900.0, 1600.0
    dragging, last_snd, last_pkt, tx_count = False, 0, "", 0
    running = True
    while running:
        clock.tick(60)
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT: running = False
            if ev.type == pygame.MOUSEBUTTONDOWN and ev.pos[0] < VIEW_W:
                dragging = True; drag_start = ev.pos; az_start, el_start = az, el
            if ev.type == pygame.MOUSEBUTTONUP: dragging = False
            if ev.type == pygame.MOUSEMOTION and dragging:
                az = az_start - (ev.pos[0] - drag_start[0]) * 0.005
                el = max(-1.5, min(1.5, el_start + (ev.pos[1] - drag_start[1]) * 0.005))
            if ev.type == pygame.MOUSEWHEEL: cam_d = max(400, min(4000, cam_d - ev.y * 80))

        now = time.monotonic()
        if now - last_snd >= (1.0 / args.hz):
            last_snd = now
            pkt = pkt_joy(joy) if joy else pkt_keys(pygame.key.get_pressed())
            last_pkt = pkt.strip()
            if S.connected:
                try: sock.sendall(pkt.encode("ascii"))
                except: S.connected = False
            tx_count += 1
            if tx_count % _TX_LOG_EVERY == 0: log.debug("TX %s", last_pkt)

        surf.fill(BG)
        with S.lock: srv, stat = [r[:] for r in S.servos], list(S.statuses)
        draw_3d(surf, (0, 0, VIEW_W, H), srv, stat, az, el, fov, cam_d)
        draw_info(surf, VIEW_W, 0, PAN_W, H, srv, stat, last_pkt, S.connected, args.ip, joy, font_sm, font_md)
        pygame.display.flip()

    pygame.quit()
    sock.close()

if __name__ == "__main__":
    main()
