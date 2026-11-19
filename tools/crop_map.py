from PIL import Image

img = Image.open("map_crop.png").convert("RGB")

rows = 16
cols = 12

w, h = img.size
cell_w = w / cols
cell_h = h / rows

def classify(rgb):
    r, g, b = rgb

    # 角色
    if g > 200 and r < 80 and b < 120:
        return "P"

    # 黄色块
    if r > 200 and g > 200 and b < 100:
        return "Y"

    # 粉色块
    if r > 200 and g < 100 and b > 200:
        return "M"

    # 蓝色空地
    if b > 200 and r < 80 and g < 80:
        return "."

    # 灰色墙体
    if abs(r - g) < 25 and abs(g - b) < 25 and 120 <= r <= 210:
        return "#"

    # 其他先记成 ?
    return "?"

char_map = []

for r in range(rows):
    row_chars = []
    for c in range(cols):
        cx = int((c + 0.5) * cell_w)
        cy = int((r + 0.5) * cell_h)

        if cx >= w:
            cx = w - 1
        if cy >= h:
            cy = h - 1

        rgb = img.getpixel((cx, cy))
        ch = classify(rgb)
        row_chars.append(ch)

    char_map.append(row_chars)

print("16x12 字符地图：")
for row in char_map:
    print(" ".join(row))