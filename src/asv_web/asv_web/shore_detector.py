#!/usr/bin/env python3
"""
ASV Shore Detector — SegFormer-B1 + Polar Radar Grid
Processes raw camera frames and overlays:
  • SegFormer-B1 semantic segmentation (water=blue, land=green, sky=grey)
  • Polar radar grid with distance arcs (1m/2m/3m/5m/12m) from bottom-centre
  • Shore boundary detection: yellow dots where land meets water in each ray
  • WALL distance + CLEAR/WARN status HUD
"""

import cv2
import numpy as np
import torch
import time
import warnings
from collections import deque

warnings.filterwarnings("ignore")

# ── Colour palette (BGR) ──────────────────────────────────────────────────────
C_CYAN    = (255, 255,   0)  # SegFormer-B1 label
C_GREEN   = (  0, 255,   0)  # grid lines, FPS, CLEAR
C_YELLOW  = (  0, 255, 255)  # WALL label, shore dots
C_ORANGE  = (  0, 165, 255)  # 2m TURN arc
C_WHITE   = (255, 255, 255)
C_RED     = (  0,   0, 255)

# ── ADE20K class colours for segmentation overlay (BGR) ──────────────────────
SEG_PALETTE = np.zeros((256, 3), dtype=np.uint8)
SEG_PALETTE[21] = [160,  45,  15]   # water      → deep blue
SEG_PALETTE[ 2] = [200, 170,  80]   # sky        → light blue-grey
SEG_PALETTE[ 9] = [ 20,  90,  20]   # grass      → dark green
SEG_PALETTE[ 0] = [ 30,  80,  30]   # terrain    → dark green
SEG_PALETTE[ 3] = [ 20,  80,  30]   # tree
SEG_PALETTE[96] = [  0,   0, 200]   # boat/ship  → red
for i in range(256):
    if SEG_PALETTE[i].sum() == 0:
        SEG_PALETTE[i] = [25, 70, 25]  # default → dark green

WATER_CLASS = 21


class ShoreDetector:
    MODEL_ID   = "nvidia/segformer-b1-finetuned-ade-512-512"
    INPUT_SIZE = (384, 384)
    OUT_W, OUT_H = 640, 360

    # Grid config
    MAX_DIST  = 12.0  # metres (visual max)
    WARN_DIST =  2.0  # orange "2m TURN" arc
    ARC_DISTS = [1, 2, 3, 5, 12]
    RAY_DEGS  = [-45, -30, -20, -10, 0, 10, 20, 30, 45]  # from vertical
    FOV_HALF  = 45    # degrees either side

    def __init__(self):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        print(f"[ShoreDetector] Loading SegFormer-B1 on {self.device}...")
        from transformers import SegformerForSemanticSegmentation
        self.model = SegformerForSemanticSegmentation.from_pretrained(self.MODEL_ID)
        self.model.eval().to(self.device).half()
        self._mean = torch.tensor([0.485, 0.456, 0.406],
                                  device=self.device, dtype=torch.float16).view(1,3,1,1)
        self._std  = torch.tensor([0.229, 0.224, 0.225],
                                  device=self.device, dtype=torch.float16).view(1,3,1,1)
        self._fps  = deque(maxlen=30)
        # Warmup
        _d = torch.zeros(1, 3, *self.INPUT_SIZE, device=self.device, dtype=torch.float16)
        for _ in range(5):
            self.model(pixel_values=_d)
        print("[ShoreDetector] Ready.")

    # ── Inference ─────────────────────────────────────────────────────────────
    @torch.no_grad()
    def _segment(self, bgr):
        rgb = cv2.cvtColor(cv2.resize(bgr, self.INPUT_SIZE), cv2.COLOR_BGR2RGB)
        t = torch.from_numpy(rgb).permute(2,0,1).unsqueeze(0).to(self.device).half()/255.
        logits = self.model(pixel_values=(t - self._mean) / self._std).logits
        pred = logits.argmax(1).squeeze(0).cpu().numpy().astype(np.uint8)
        return cv2.resize(pred, (bgr.shape[1], bgr.shape[0]),
                          interpolation=cv2.INTER_NEAREST)

    # ── Polar grid ────────────────────────────────────────────────────────────
    def _draw_grid(self, canvas):
        H, W = canvas.shape[:2]
        cx, cy = W // 2, H          # apex = bottom-centre (just below frame)
        max_r  = int(H * 0.92)      # farthest arc just within frame

        # Yellow outer box
        cv2.rectangle(canvas, (1, 1), (W-2, H-2), C_YELLOW, 2)

        # Radial lines
        for deg in self.RAY_DEGS:
            rad = np.radians(deg)
            dx, dy = np.sin(rad), -np.cos(rad)
            ex = int(cx + dx * max_r * 1.5)
            ey = int(cy + dy * max_r * 1.5)
            ex = np.clip(ex, 0, W-1)
            ey = np.clip(ey, 0, H-1)
            thick = 2 if deg == 0 else 1
            cv2.line(canvas, (cx, cy), (ex, ey), C_GREEN, thick, cv2.LINE_AA)
            if deg != 0:
                lx = np.clip(int(cx + dx * max_r * 0.95), 4, W-20)
                ly = np.clip(int(cy + dy * max_r * 0.95), 10, H-4)
                label = f"+{deg}" if deg > 0 else str(deg)
                cv2.putText(canvas, label, (lx, ly),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.32, C_GREEN, 1, cv2.LINE_AA)

        # Arcs
        for d in self.ARC_DISTS:
            r     = int(d / self.MAX_DIST * max_r)
            color = C_ORANGE if d == self.WARN_DIST else C_GREEN
            thick = 2       if d == self.WARN_DIST else 1
            start_a = 270 - self.FOV_HALF   # 225
            end_a   = 270 + self.FOV_HALF   # 315
            cv2.ellipse(canvas, (cx, cy), (r, r), 0,
                        start_a, end_a, color, thick, cv2.LINE_AA)
            # Distance label on centre vertical
            ly = cy - r - 4
            if 4 < ly < H - 4:
                label = f"{d}m" + (" TURN" if d == self.WARN_DIST else "")
                cv2.putText(canvas, label, (cx + 5, ly),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.38, color, 1, cv2.LINE_AA)

    # ── Shore detection ───────────────────────────────────────────────────────
    def _detect_shore(self, water_mask):
        H, W = water_mask.shape
        cx, cy = W // 2, H
        max_r  = int(H * 0.92)
        N_RAYS = 80

        shore_pts, min_dist, min_pt = [], self.MAX_DIST, None

        for i in range(N_RAYS):
            deg = -self.FOV_HALF + i * (2 * self.FOV_HALF) / (N_RAYS - 1)
            rad = np.radians(deg)
            dx, dy = np.sin(rad), -np.cos(rad)

            for step in range(2, 101):
                r = step / 100 * max_r
                px, py = int(cx + dx * r), int(cy + dy * r)
                if px < 0 or px >= W or py < 0 or py >= H:
                    break
                if not water_mask[py, px]:
                    dist = (r / max_r) * self.MAX_DIST
                    shore_pts.append((px, py, dist))
                    if dist < min_dist:
                        min_dist, min_pt = dist, (px, py)
                    break

        return shore_pts, min_dist, min_pt

    # ── Main pipeline ─────────────────────────────────────────────────────────
    def process_jpeg(self, jpeg_bytes: bytes) -> bytes:
        t0 = time.perf_counter()

        # Decode
        arr   = np.frombuffer(jpeg_bytes, np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if frame is None:
            return jpeg_bytes

        OW, OH = self.OUT_W, self.OUT_H
        frame  = cv2.resize(frame, (OW, OH))

        # ── Segmentation ─────────────────────────────────────────────────────
        pred     = self._segment(frame)
        seg_col  = SEG_PALETTE[pred]
        water_m  = (pred == WATER_CLASS)
        overlay  = cv2.addWeighted(frame, 0.30, seg_col, 0.70, 0)

        # ── Grid & shore detection ────────────────────────────────────────────
        self._draw_grid(overlay)
        shore_pts, min_dist, min_pt = self._detect_shore(water_m)

        for px, py, _ in shore_pts:
            cv2.circle(overlay, (px, py), 3, C_YELLOW, -1, cv2.LINE_AA)
        if min_pt:
            cv2.circle(overlay, min_pt, 7, C_YELLOW, -1, cv2.LINE_AA)
            cv2.putText(overlay, "shore", (min_pt[0]+9, min_pt[1]-5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, C_YELLOW, 1, cv2.LINE_AA)

        # ── HUD ───────────────────────────────────────────────────────────────
        dt = time.perf_counter() - t0
        self._fps.append(dt)
        fps = 1.0 / (sum(self._fps) / len(self._fps))

        warn        = min_dist < self.WARN_DIST
        status_col  = C_RED if warn else C_GREEN
        status_txt  = "WARN" if warn else "CLEAR"
        wall_txt    = f"WALL: {min_dist:.1f} m"

        # black semi-transparent top-left bar
        bar = overlay[:52, :].copy()
        cv2.rectangle(overlay, (0,0), (OW, 52), (0,0,0), -1)
        cv2.addWeighted(bar, 0.25, overlay[:52,:], 0.75, 0, overlay[:52,:])

        cv2.putText(overlay, "SegFormer-B1", ( 8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.70, C_CYAN,   2, cv2.LINE_AA)
        cv2.putText(overlay, f"FPS: {fps:.2f}",( 8, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.60, C_GREEN,  1, cv2.LINE_AA)
        cv2.putText(overlay, wall_txt,   (OW-170, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.60, C_YELLOW, 1, cv2.LINE_AA)
        cv2.putText(overlay, status_txt, (OW-100, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.60, status_col,1, cv2.LINE_AA)

        _, enc = cv2.imencode('.jpg', overlay, [cv2.IMWRITE_JPEG_QUALITY, 78])
        return enc.tobytes()
