"""Draws the flight controller Tuner icon (quad-X drone on a green tile) -> app.ico. Needs Pillow."""
import sys
from PIL import Image, ImageDraw

out = sys.argv[1] if len(sys.argv) > 1 else "app.ico"
S = 256
im = Image.new("RGBA", (S, S), (0, 0, 0, 0))
d = ImageDraw.Draw(im)
d.rounded_rectangle([8, 8, S - 8, S - 8], radius=48, fill=(46, 125, 70, 255))
c = S / 2
r = 70                                   # arm length to rotor centre
w = 16
for sx, sy in [(-1, -1), (1, -1), (-1, 1), (1, 1)]:
    x, y = c + sx * r, c + sy * r
    d.line([c, c, x, y], fill=(255, 255, 255, 255), width=w)
    d.ellipse([x - 34, y - 34, x + 34, y + 34], outline=(255, 255, 255, 255), width=10)
d.rounded_rectangle([c - 26, c - 26, c + 26, c + 26], radius=10, fill=(255, 255, 255, 255))
d.ellipse([c - 9, c - 9, c + 9, c + 9], fill=(46, 125, 70, 255))
im.save(out, sizes=[(16, 16), (24, 24), (32, 32), (48, 48), (64, 64), (128, 128), (256, 256)])
print("wrote", out)
