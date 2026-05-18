#!/usr/bin/env python3
"""
hexapod_calibrator.py — Individual Servo Calibration Tool
PS5 controller not required. Uses sliders to adjust all 18 servos.
Connects via Tailscale to the Pi Relay.
"""

import argparse, logging, math, os, re, sys, threading, time, socket
from collections import deque
from datetime import datetime
import pygame

# ─── File logger ─────────────────────────────────────────────────────────────
_LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
os.makedirs(_LOG_DIR, exist_ok=True)
_log_path = os.path.join(_LOG_DIR, datetime.now().strftime("calibrator_%Y%m%d_%H%M%S.log"))
logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s", handlers=[logging.FileHandler(_log_path), logging.StreamHandler()])
log = logging.getLogger("calibrator")

# ─── Robot geometry (for 3D preview) ──────────────────────────────────────────
L1, L2, L3 = 66.93, 89.80, 165.56
RAD2DEG = 57.29578
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
        self.servos    = [90.0] * 18
        self.connected = False
        self.enabled   = False

S = State()

def network_thread(sock):
    while True:
        try:
            data = sock.recv(4096)
            if not data:
                S.connected = False
                break
            S.connected = True
        except:
            S.connected = False
            break

# ─── UI Helper Classes ────────────────────────────────────────────────────────
class Slider:
    def __init__(self, x, y, w, h, label, min_val, max_val, initial_val):
        self.rect = pygame.Rect(x, y, w, h)
        self.label = label
        self.min_val = min_val
        self.max_val = max_val
        self.val = initial_val
        self.grabbed = False

    def draw(self, surf, font):
        pygame.draw.rect(surf, (40, 40, 50), self.rect)
        px = self.rect.x + int((self.val - self.min_val) / (self.max_val - self.min_val) * self.rect.width)
        handle_rect = pygame.Rect(px - 5, self.rect.y - 2, 10, self.rect.height + 4)
        pygame.draw.rect(surf, (200, 200, 220), handle_rect)
        txt = font.render(f"{self.label}: {int(self.val)}", True, (210, 210, 215))
        surf.blit(txt, (self.rect.x, self.rect.y - 18))

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            if self.rect.collidepoint(event.pos):
                self.grabbed = True
        if event.type == pygame.MOUSEBUTTONUP:
            self.grabbed = False
        if event.type == pygame.MOUSEMOTION and self.grabbed:
            rel_x = max(0, min(self.rect.width, event.pos[0] - self.rect.x))
            self.val = self.min_val + (rel_x / self.rect.width) * (self.max_val - self.min_val)
            return True
        return False

# ─── 3-D Camera & Drawing (Reused) ────────────────────────────────────────────
def _cam(pt, az, el):
    x, y, z = pt
    cz, sz = math.cos(az), math.sin(az); x, y = x*cz - y*sz, x*sz + y*cz
    ce, se = math.cos(el), math.sin(el); y, z = y*ce + z*se, -y*se + z*ce
    return x, y, z

def _proj(cam_pt, ox, oy, fov, cam_d):
    x, y, z = cam_pt; depth = max(cam_d - y, 10.0); s = fov / depth
    return int(ox + x * s), int(oy - z * s), depth

BG=(14,14,20); C_GRID=(30,30,42); C_BODY=(55,75,120); C_COXA=(75,75,115); C_OK=(55,195,95); C_FOOT=(255,225,70); C_PANEL=(20,20,28)

def draw_3d(surf, view_rect, servos, az, el, fov, cam_d):
    vx, vy, vw, vh = view_rect; ox, oy = vx + vw // 2, vy + vh // 2
    def P(pt): return _proj(_cam(pt, az, el), ox, oy, fov, cam_d)
    segs, dots, fills = [], [], []; gz = -BODY_HEIGHT_MM
    for gv in range(-600, 601, 100):
        for a, b in [((gv,-600,gz),(gv,600,gz)), ((-600,gv,gz),(600,gv,gz))]:
            sx1,sy1,d1 = P(a); sx2,sy2,d2 = P(b)
            segs.append(((d1+d2)/2, C_GRID, 1, (sx1,sy1), (sx2,sy2)))
    btop_pts = [(PIVOT_X[i], PIVOT_Y[i], 20.0) for i in range(6)]
    proj_top = [P(p) for p in btop_pts]
    fills.append((sum(p[2] for p in proj_top)/6, C_BODY, [(p[0],p[1]) for p in proj_top]))
    for i in range(6):
        # Index mapping: Right board (0-8), Left board (9-17)
        if i < 3: # FR, MR, BR
            c, f, t = servos[i*3], servos[i*3+1], servos[i*3+2]
        else: # BL, ML, FL
            c, f, t = servos[9+(i-3)*3], servos[10+(i-3)*3], servos[11+(i-3)*3]
        t1,t2,t3 = servo_to_theta(i, c, f, t)
        p0,p1,p2,p3 = fk3(i, t1, t2, t3)
        sp0, sp1, sp2, sp3 = P(p0), P(p1), P(p2), P(p3)
        segs.append(((sp0[2]+sp1[2])/2, C_COXA, 2, sp0[:2], sp1[:2]))
        segs.append(((sp1[2]+sp2[2])/2, C_OK, 3, sp1[:2], sp2[:2]))
        segs.append(((sp2[2]+sp3[2])/2, C_OK, 3, sp2[:2], sp3[:2]))
        dots.append((sp3[2], C_FOOT, sp3[:2], 6))
    all_draw = ([(d,'fill',c,pts) for d,c,pts in fills] + [(d,'seg',c,w,p1,p2) for d,c,w,p1,p2 in segs] + [(d,'dot',c,pos,r) for d,c,pos,r in dots])
    all_draw.sort(key=lambda x: -x[0])
    for item in all_draw:
        if item[1] == 'fill': pygame.draw.polygon(surf, item[2], item[3])
        elif item[1] == 'seg': pygame.draw.line(surf, item[2], item[4], item[5], item[3])
        else: pygame.draw.circle(surf, item[2], item[3], item[4])

# ─── Main ─────────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ip", default="100.122.9.49")
    ap.add_argument("--port", default=5000, type=int)
    args = ap.parse_args()

    pygame.init()
    W, H = 1400, 800
    VIEW_W = 800
    surf = pygame.display.set_mode((W, H))
    pygame.display.set_caption("Hexapod Servo Calibrator")
    font = pygame.font.SysFont("monospace", 14)
    clock = pygame.time.Clock()

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.connect((args.ip, args.port))
        S.connected = True
        threading.Thread(target=network_thread, args=(sock,), daemon=True).start()
    except Exception as e:
        log.error(f"Failed to connect: {e}")

    sliders = []
    # Right Board
    for i in range(9):
        leg = NAMES[i//3]
        joint = ["Coxa", "Femur", "Tibia"][i%3]
        sliders.append(Slider(850, 50 + i*75, 200, 15, f"R {leg} {joint}", 0, 180, 90))
    # Left Board
    for i in range(9):
        leg = NAMES[3 + i//3]
        joint = ["Coxa", "Femur", "Tibia"][i%3]
        sliders.append(Slider(1150, 50 + i*75, 200, 15, f"L {leg} {joint}", 0, 180, 90))

    az, el, fov, cam_d = math.radians(30), math.radians(35), 900.0, 1600.0
    dragging = False
    running = True

    while running:
        clock.tick(60)
        changed = False
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT: running = False
            if ev.type == pygame.KEYDOWN and ev.key == pygame.K_e:
                if S.connected:
                    sock.sendall(b"/SERVO/EN\n")
                    S.enabled = True
                    log.info("Sent Servo Enable")
            if ev.type == pygame.MOUSEBUTTONDOWN and ev.pos[0] < VIEW_W:
                dragging = True; drag_start = ev.pos; az_start, el_start = az, el
            if ev.type == pygame.MOUSEBUTTONUP: dragging = False
            if ev.type == pygame.MOUSEMOTION and dragging:
                az = az_start - (ev.pos[0] - drag_start[0]) * 0.005
                el = max(-1.5, min(1.5, el_start + (ev.pos[1] - drag_start[1]) * 0.005))
            if ev.type == pygame.MOUSEWHEEL: cam_d = max(400, min(4000, cam_d - ev.y * 80))
            
            for s in sliders:
                if s.handle_event(ev): changed = True

        if changed and S.connected:
            vals = ",".join([f"{int(s.val)}" for s in sliders])
            sock.sendall(f"/SERVO/{vals}\n".encode("ascii"))
            with S.lock: S.servos = [s.val for s in sliders]

        surf.fill(BG)
        draw_3d(surf, (0, 0, VIEW_W, H), S.servos, az, el, fov, cam_d)
        
        pygame.draw.rect(surf, C_PANEL, (VIEW_W, 0, W-VIEW_W, H))
        for s in sliders: s.draw(surf, font)
        
        status_txt = font.render(f"Connected: {S.connected} | Enabled: {S.enabled} | Press 'E' to Enable Servos", True, C_OK if S.connected else (215, 50, 50))
        surf.blit(status_txt, (VIEW_W + 10, H - 30))
        
        pygame.display.flip()

    pygame.quit()
    sock.close()

if __name__ == "__main__":
    main()
