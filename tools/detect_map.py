from PIL import Image
import os
import time

# =========================
# 1. 文件夹设置
# =========================
INPUT_DIR = "input"
OUTPUT_DIR = "output"

# =========================
# 2. 裁剪参数
# 从整张截图里裁出左侧地图区域
# 这组参数基于你当前这类截图
# =========================
CROP_X1 = 0
CROP_Y1 = 0
CROP_X2 = 540
CROP_Y2 = 720

# =========================
# 3. 地图规格
# 现在是 12 行 16 列
# =========================
ROWS = 12
COLS = 16

# =========================
# 4. 支持的图片格式
# =========================
VALID_EXTS = (".png", ".jpg", ".jpeg", ".bmp")

# =========================
# 5. 已处理文件记录
# =========================
processed_files = set()


def classify(rgb):
    r, g, b = rgb

    # 人物 @
    if g > 200 and r < 80 and b < 120:
        return "@"

    # 箱子 $
    if r > 200 and g < 100 and b > 200:
        return "$"

    # 目标点 .
    if r > 200 and g > 200 and b < 100:
        return "."

    # 空地 -
    if b > 200 and r < 80 and g < 80:
        return "-"

    # 墙体 #
    if abs(r - g) < 25 and abs(g - b) < 25 and 120 <= r <= 210:
        return "#"

    # 其他未知，先按空地处理，避免输出问号
    return "-"


def detect_map(image_path):
    img = Image.open(image_path).convert("RGB")

    # 裁剪地图区域
    map_img = img.crop((CROP_X1, CROP_Y1, CROP_X2, CROP_Y2))

    w, h = map_img.size
    cell_w = w / COLS
    cell_h = h / ROWS

    char_map = []

    for r in range(ROWS):
        row_chars = []
        for c in range(COLS):
            cx = int((c + 0.5) * cell_w)
            cy = int((r + 0.5) * cell_h)

            if cx >= w:
                cx = w - 1
            if cy >= h:
                cy = h - 1

            rgb = map_img.getpixel((cx, cy))
            ch = classify(rgb)
            row_chars.append(ch)

        char_map.append(row_chars)

    return char_map


def save_txt(char_map, txt_path):
    with open(txt_path, "w", encoding="utf-8") as f:
        for row in char_map:
            f.write("".join(row) + "\n")


def process_one_image(filename):
    image_path = os.path.join(INPUT_DIR, filename)
    name, _ = os.path.splitext(filename)
    txt_path = os.path.join(OUTPUT_DIR, name + ".txt")

    try:
        char_map = detect_map(image_path)
        save_txt(char_map, txt_path)

        print(f"已处理: {filename}")
        print(f"已输出: {txt_path}")
        for row in char_map:
            print("".join(row))
        print("-" * 30)

    except Exception as e:
        print(f"处理失败: {filename}")
        print(f"错误: {e}")
        print("-" * 30)


def main():
    os.makedirs(INPUT_DIR, exist_ok=True)
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    print("程序已启动")
    print(f"把截图放进 {INPUT_DIR} 文件夹，程序会自动生成 txt 地图到 {OUTPUT_DIR} 文件夹")
    print("当前输出格式为官方字符风格: # - . $ @")
    print("按 Ctrl + C 停止程序")
    print("-" * 30)

    while True:
        files = os.listdir(INPUT_DIR)

        for filename in files:
            if not filename.lower().endswith(VALID_EXTS):
                continue

            if filename in processed_files:
                continue

            process_one_image(filename)
            processed_files.add(filename)

        time.sleep(1)


if __name__ == "__main__":
    main()