"""
LSM6DSM 3D Orientation Visualizer
===================================
STM32'den UART üzerinden gelen CSV veriyi okuyarak
3D bir cismin (IMU kartı) gerçek zamanlı dönüşünü gösterir.

Format: "ROLL,PITCH,YAW\n"  (derece)

Gereksinimler:
    pip install pyserial pygame PyOpenGL PyOpenGL_accelerate

Kullanım:
    python visualizer_3d.py --port COM3 --baud 115200
"""

import argparse
import threading
import time
import math

import serial
import pygame
from pygame.locals import DOUBLEBUF, OPENGL, QUIT, KEYDOWN, K_ESCAPE, K_r
from OpenGL.GL import (
    glBegin, glEnd, glVertex3f, glColor3f, glNormal3f,
    glClear, glLoadIdentity, glTranslatef, glRotatef,
    glEnable, glShadeModel, glLightfv, glMaterialfv,
    glColorMaterial, glBlendFunc, glLineWidth,
    GL_COLOR_BUFFER_BIT, GL_DEPTH_BUFFER_BIT,
    GL_DEPTH_TEST, GL_LIGHTING, GL_LIGHT0, GL_LIGHT1,
    GL_SMOOTH, GL_POSITION, GL_DIFFUSE, GL_AMBIENT, GL_SPECULAR,
    GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE,
    GL_COLOR_MATERIAL, GL_LINES, GL_QUADS,
    GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_BLEND,
)
from OpenGL.GLU import gluPerspective
import OpenGL.GL as gl

# ── Ayarlar ────────────────────────────────────────────────────────────────────
BAUD_DEFAULT   = 115200
PORT_DEFAULT   = "COM3"
WIN_W, WIN_H   = 900, 650

# ── Paylaşılan sensör durumu ───────────────────────────────────────────────────
angles = {
    "roll":  0.0,
    "pitch": 0.0,
    "yaw":   0.0,
    "connected": False,
}
sensor_lock = threading.Lock()
running = True


# ══════════════════════════════════════════════════════════════════════════════
#  Serial okuma thread'i
# ══════════════════════════════════════════════════════════════════════════════
def serial_reader(port: str, baud: int):
    global running
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
        ser.reset_input_buffer()
        print(f"[OK] Port açıldı: {port} @ {baud} baud")
        with sensor_lock:
            angles["connected"] = True
    except serial.SerialException as e:
        print(f"[HATA] Port açılamadı: {e}")
        print("       Demo modu: sensör simüle ediliyor...")
        running = True
        _demo_mode()
        return

    while running:
        try:
            line = ser.readline().decode("ascii", errors="ignore").strip()
            if not line:
                continue
            parts = line.split(",")
            if len(parts) != 3:
                continue
            v = [float(p) for p in parts]
        except (ValueError, serial.SerialException):
            continue

        with sensor_lock:
            angles["roll"]  = v[0]
            angles["pitch"] = v[1]
            angles["yaw"]   = v[2]

    ser.close()


def _demo_mode():
    """Port yoksa sinüs dalgasıyla demo döngüsü."""
    t = 0.0
    while running:
        t += 0.02
        with sensor_lock:
            angles["roll"]  = math.sin(t * 0.5) * 30.0
            angles["pitch"] = math.sin(t * 0.3) * 20.0
            angles["yaw"]   = t * 10.0 % 360.0
        time.sleep(0.010)


# ══════════════════════════════════════════════════════════════════════════════
#  OpenGL çizim fonksiyonları
# ══════════════════════════════════════════════════════════════════════════════
# Kart boyutları (birim küp benzeri ama yassı)
BW, BH, BD = 1.6, 0.08, 1.0   # Genişlik, Yükseklik, Derinlik

FACES = [
    # (normal, renk, 4 köşe)
    # Üst yüz – yeşil (PCB rengi)
    ([0, 1, 0], [0.15, 0.55, 0.25],
     [(-BW, BH, -BD), (BW, BH, -BD), (BW, BH, BD), (-BW, BH, BD)]),
    # Alt yüz – koyu gri
    ([0, -1, 0], [0.2, 0.2, 0.2],
     [(-BW, -BH, BD), (BW, -BH, BD), (BW, -BH, -BD), (-BW, -BH, -BD)]),
    # Ön yüz – sarı
    ([0, 0, 1], [0.85, 0.75, 0.1],
     [(-BW, -BH, BD), (BW, -BH, BD), (BW, BH, BD), (-BW, BH, BD)]),
    # Arka yüz – mavi
    ([0, 0, -1], [0.15, 0.35, 0.75],
     [(BW, -BH, -BD), (-BW, -BH, -BD), (-BW, BH, -BD), (BW, BH, -BD)]),
    # Sağ yüz – kırmızı
    ([1, 0, 0], [0.8, 0.15, 0.15],
     [(BW, -BH, BD), (BW, -BH, -BD), (BW, BH, -BD), (BW, BH, BD)]),
    # Sol yüz – açık mavi
    ([-1, 0, 0], [0.15, 0.65, 0.85],
     [(-BW, -BH, -BD), (-BW, -BH, BD), (-BW, BH, BD), (-BW, BH, -BD)]),
]


def draw_board():
    glBegin(GL_QUADS)
    for normal, color, verts in FACES:
        glNormal3f(*normal)
        glColor3f(*color)
        for v in verts:
            glVertex3f(*v)
    glEnd()


def draw_axes():
    """Küçük RGB eksen göstergesi."""
    glLineWidth(2.5)
    glBegin(GL_LINES)
    glColor3f(1, 0, 0); glVertex3f(0,0,0); glVertex3f(2.2, 0,   0)    # X kırmızı
    glColor3f(0, 1, 0); glVertex3f(0,0,0); glVertex3f(0,   2.2, 0)    # Y yeşil
    glColor3f(0, 0, 1); glVertex3f(0,0,0); glVertex3f(0,   0,   2.2)  # Z mavi
    glEnd()
    glLineWidth(1.0)


def draw_grid():
    """Zemin ızgara çizgisi."""
    glColor3f(0.25, 0.25, 0.25)
    glLineWidth(1.0)
    glBegin(GL_LINES)
    for i in range(-5, 6):
        glVertex3f(i * 0.8, -2.5, -4.0)
        glVertex3f(i * 0.8, -2.5,  4.0)
        glVertex3f(-4.0, -2.5, i * 0.8)
        glVertex3f( 4.0, -2.5, i * 0.8)
    glEnd()


def setup_lighting():
    glEnable(GL_LIGHTING)
    glEnable(GL_LIGHT0)
    glEnable(GL_LIGHT1)
    glEnable(GL_COLOR_MATERIAL)
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
    glShadeModel(GL_SMOOTH)

    glLightfv(GL_LIGHT0, GL_POSITION, [4.0, 6.0, 4.0, 1.0])
    glLightfv(GL_LIGHT0, GL_DIFFUSE,  [1.0, 1.0, 1.0, 1.0])
    glLightfv(GL_LIGHT0, GL_AMBIENT,  [0.25, 0.25, 0.25, 1.0])

    glLightfv(GL_LIGHT1, GL_POSITION, [-4.0, -3.0, -4.0, 1.0])
    glLightfv(GL_LIGHT1, GL_DIFFUSE,  [0.4, 0.4, 0.5, 1.0])


# ── Pygame üzerine metin ───────────────────────────────────────────────────────
def draw_text_overlay(screen, font, roll, pitch, yaw, connected):
    lines = [
        f"Roll  (X): {roll:+7.1f}°",
        f"Pitch (Y): {pitch:+7.1f}°",
        f"Yaw   (Z): {yaw:+7.1f}°",
        "",
        "R  →  Sıfırla",
        "ESC →  Çıkış",
    ]
    status_color = (80, 220, 80) if connected else (220, 180, 60)
    status_text  = "● BAĞLI" if connected else "● DEMO"

    x, y = 14, 14
    pygame.draw.rect(screen, (20, 20, 20, 180), (8, 8, 195, 160), border_radius=8)
    surf = font.render(status_text, True, status_color)
    screen.blit(surf, (x, y)); y += 22

    for ln in lines:
        surf = font.render(ln, True, (220, 220, 220))
        screen.blit(surf, (x, y)); y += 20


# ══════════════════════════════════════════════════════════════════════════════
#  Ana döngü
# ══════════════════════════════════════════════════════════════════════════════
def main():
    global running

    parser = argparse.ArgumentParser()
    parser.add_argument("--port",  default=PORT_DEFAULT)
    parser.add_argument("--baud",  default=BAUD_DEFAULT, type=int)
    args = parser.parse_args()

    # Serial thread
    t = threading.Thread(target=serial_reader, args=(args.port, args.baud), daemon=True)
    t.start()

    # Pygame + OpenGL
    pygame.init()
    screen = pygame.display.set_mode((WIN_W, WIN_H), DOUBLEBUF | OPENGL)
    pygame.display.set_caption("LSM6DSM – 3D Orientation Visualizer")
    font = pygame.font.SysFont("Consolas", 15, bold=False)

    gluPerspective(45, WIN_W / WIN_H, 0.1, 50.0)
    glTranslatef(0.0, 0.0, -5.5)
    glEnable(GL_DEPTH_TEST)
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    setup_lighting()

    clock = pygame.time.Clock()
    yaw_offset = 0.0   # R tuşu ile yaw sıfırlama referansı

    while running:
        for event in pygame.event.get():
            if event.type == QUIT:
                running = False
            elif event.type == KEYDOWN:
                if event.key == K_ESCAPE:
                    running = False
                elif event.key == K_r:
                    with sensor_lock:
                        yaw_offset = angles["yaw"]

        # Sensör verisi oku
        with sensor_lock:
            roll      = angles["roll"]
            pitch     = angles["pitch"]
            yaw       = angles["yaw"] - yaw_offset
            connected = angles["connected"]

        # ── OpenGL çizimi ────────────────────────────────────────────────
        gl.glClearColor(0.08, 0.08, 0.12, 1.0)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        draw_grid()
        draw_axes()

        glLoadIdentity()
        gluPerspective(45, WIN_W / WIN_H, 0.1, 50.0)
        glTranslatef(0.0, 0.0, -5.5)

        # Dönüşleri uygula: yaw (Y), pitch (X, terslenmiş), roll (Z)
        glRotatef(yaw,    0, 1, 0)
        glRotatef(pitch, 0, 0, 1)
        glRotatef(-roll,   1, 0, 0)

        draw_board()

        # ── 2D metin katmanı (pygame blitting) ───────────────────────────
        # Metin için ayrı surface (OpenGL üzerine blit)
        text_surf = pygame.Surface((WIN_W, WIN_H), pygame.SRCALPHA)
        text_surf.fill((0, 0, 0, 0))
        draw_text_overlay(text_surf, font, roll, pitch, yaw, connected)

        # OpenGL içine pygame yüzeyi çizme
        text_data = pygame.image.tostring(text_surf, "RGBA", True)
        gl.glWindowPos2d(0, 0)
        gl.glDrawPixels(WIN_W, WIN_H, gl.GL_RGBA, gl.GL_UNSIGNED_BYTE, text_data)

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()
    print("[OK] Kapatıldı.")


if __name__ == "__main__":
    main()
