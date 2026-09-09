"""
A very small PNG writer and raster canvas.

The fleet has to render maps and traces on a Raspberry Pi where Pillow and
numpy may not be installed, so this does the few things needed by hand: an
8-bit RGB PNG, and enough drawing primitives to plot an occupancy grid with
paths and robots on top of it. Standard library only.
"""

import math
import struct
import zlib


def write_png(path, width, height, rows):
    """Write an 8-bit RGB PNG. `rows` is a sequence of `bytearray`s of length 3*width."""
    raw = bytearray()
    for row in rows:
        raw.append(0)          # filter type 0 (None) for every scanline
        raw.extend(row)

    def chunk(tag, payload):
        body = tag + payload
        return (struct.pack(">I", len(payload)) + body
                + struct.pack(">I", zlib.crc32(body) & 0xFFFFFFFF))

    header = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    png = (b"\x89PNG\r\n\x1a\n"
           + chunk(b"IHDR", header)
           + chunk(b"IDAT", zlib.compress(bytes(raw), 6))
           + chunk(b"IEND", b""))
    with open(path, "wb") as fh:
        fh.write(png)
    return len(png)


class Canvas:
    """An RGB raster with the origin at the top-left."""

    def __init__(self, width, height, background=(255, 255, 255)):
        self.w, self.h = width, height
        self.rows = [bytearray(bytes(background) * width) for _ in range(height)]

    def set(self, x, y, colour):
        if 0 <= x < self.w and 0 <= y < self.h:
            i = 3 * x
            self.rows[y][i:i + 3] = bytes(colour)

    def rect(self, x0, y0, x1, y1, colour):
        x0, x1 = max(0, min(x0, x1)), min(self.w - 1, max(x0, x1))
        y0, y1 = max(0, min(y0, y1)), min(self.h - 1, max(y0, y1))
        span = bytes(colour) * max(0, x1 - x0 + 1)
        for y in range(y0, y1 + 1):
            self.rows[y][3 * x0:3 * (x1 + 1)] = span

    def line(self, x0, y0, x1, y1, colour, width=1):
        dx, dy = abs(x1 - x0), -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        r = width // 2
        while True:
            if width <= 1:
                self.set(x0, y0, colour)
            else:
                self.rect(x0 - r, y0 - r, x0 + r, y0 + r, colour)
            if x0 == x1 and y0 == y1:
                return
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x0 += sx
            if e2 <= dx:
                err += dx
                y0 += sy

    def disc(self, cx, cy, radius, colour):
        r2 = radius * radius
        for y in range(-radius, radius + 1):
            span = int((r2 - y * y) ** 0.5) if r2 >= y * y else 0
            self.rect(cx - span, cy + y, cx + span, cy + y, colour)

    def ring(self, cx, cy, radius, colour, width=1):
        for t in range(0, 360, 2):
            a = math.radians(t)
            x = cx + int(radius * math.cos(a))
            y = cy + int(radius * math.sin(a))
            if width <= 1:
                self.set(x, y, colour)
            else:
                self.disc(x, y, width // 2, colour)

    def save(self, path):
        return write_png(path, self.w, self.h, self.rows)
