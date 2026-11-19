import sensor
import image
import time
import os

# OpenArt Plus 曝光测试（兼容/诊断版）
#
# 优先保存到：
#   板端 /exposure_sweep
#   Windows E:\exposure_sweep
#
# 如果板端无法写子目录，则退到板载根目录，文件名以 exp_ 开头：
#   Windows E:\exp_00_auto.jpg 等

# 第一轮粗扫已经确认：600~1000 us 区间效果最好。
# 本轮只细扫附近范围，用于确定最终固定曝光值。
# get_exposure_us() 在当前固件中返回 0，不影响设置值实际生效。
EXPOSURE_VALUES_US = (
    600,
    700,
    800,
    850,
    875,
    900,
    925,
    950,
    975,
    1000,
    1025,
    1050,
    1100,
    1200,
)

SETTLE_FRAMES = 6


def safe_read(function_name, default_value):
    try:
        return getattr(sensor, function_name)()
    except Exception as error:
        print("[WARN]", function_name, "unsupported:", error)
        return default_value


def safe_set_auto_exposure(enabled, exposure_us=None):
    try:
        if exposure_us is None:
            sensor.set_auto_exposure(enabled)
        else:
            sensor.set_auto_exposure(enabled, exposure_us=exposure_us)
        return True
    except Exception as first_error:
        # 部分 OpenArt/OpenMV 兼容固件只接受位置参数。
        try:
            if exposure_us is None:
                sensor.set_auto_exposure(enabled)
            else:
                sensor.set_auto_exposure(enabled, exposure_us)
            return True
        except Exception as second_error:
            print("[ERROR] set_auto_exposure failed:",
                  first_error, second_error)
            return False


def safe_set_auto_gain(enabled, gain_db=None):
    try:
        if gain_db is None:
            sensor.set_auto_gain(enabled)
        else:
            sensor.set_auto_gain(enabled, gain_db=gain_db)
        return True
    except Exception as error:
        print("[WARN] set_auto_gain failed:", error)
        return False


def safe_set_auto_whitebal(enabled, rgb_gain_db=None):
    try:
        if rgb_gain_db is None:
            sensor.set_auto_whitebal(enabled)
        else:
            sensor.set_auto_whitebal(
                enabled,
                rgb_gain_db=rgb_gain_db
            )
        return True
    except Exception as error:
        print("[WARN] set_auto_whitebal failed:", error)
        return False


def settle_camera(frame_count):
    img = None
    for _ in range(frame_count):
        img = sensor.snapshot()
        time.sleep_ms(50)
    return img


def path_join(folder, filename):
    if folder == "/":
        return "/" + filename
    return folder + "/" + filename


def test_write_folder(folder):
    test_path = path_join(folder, "_write_test.txt")
    try:
        with open(test_path, "w") as file:
            file.write("ok")
        os.remove(test_path)
        return True
    except Exception as error:
        print("[WARN] folder not writable:", folder, error)
        return False


def choose_output():
    # 先尝试现有子目录，不在板端强制 mkdir，避免文件系统兼容问题。
    if test_write_folder("/exposure_sweep"):
        return "/exposure_sweep", ""

    # 再尝试板载根目录。为了不污染 main.py 等文件，增加 exp_ 前缀。
    if test_write_folder("/"):
        return "/", "exp_"

    raise OSError("No writable board filesystem")


def save_photo(img, path):
    # OpenArt Plus 某些固件不支持 quality= 参数，使用最基础的 save。
    print("[SAVE]", path)
    img.save(path)


def save_results(path, lines):
    print("[SAVE]", path)
    with open(path, "w") as file:
        for line in lines:
            file.write(line + "\n")


print("[1/7] camera reset")
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)

print("[2/7] enable automatic controls")
safe_set_auto_exposure(True)
safe_set_auto_gain(True)
safe_set_auto_whitebal(True)
sensor.skip_frames(time=3000)

print("[3/7] select writable output")
OUTPUT_DIR, FILE_PREFIX = choose_output()
print("[INFO] output:", OUTPUT_DIR, "prefix:", FILE_PREFIX)

print("[4/7] capture automatic reference")
auto_img = settle_camera(SETTLE_FRAMES)
auto_exposure_us = safe_read("get_exposure_us", -1)
auto_gain_db = safe_read("get_gain_db", None)
auto_rgb_gain_db = safe_read("get_rgb_gain_db", None)

# 某些兼容固件不抛异常，而是用 -1 表示“不支持读取”。
if auto_gain_db is not None and auto_gain_db < 0:
    auto_gain_db = None

if auto_rgb_gain_db is not None:
    try:
        if min(auto_rgb_gain_db) < 0:
            auto_rgb_gain_db = None
    except Exception:
        auto_rgb_gain_db = None

auto_stat = auto_img.get_statistics()

auto_name = FILE_PREFIX + "00_auto.jpg"
save_photo(auto_img, path_join(OUTPUT_DIR, auto_name))

results = [
    "auto_exposure_us={}".format(auto_exposure_us),
    "auto_gain_db={}".format(auto_gain_db),
    "auto_rgb_gain_db={}".format(auto_rgb_gain_db),
    "{} exposure_us={} L_mean={} L_stdev={}".format(
        auto_name,
        auto_exposure_us,
        auto_stat.l_mean(),
        auto_stat.l_stdev()
    )
]

print("[5/7] lock gain and white balance when readable")
if auto_gain_db is None:
    print("[WARN] invalid gain readback; keep auto gain enabled")
else:
    safe_set_auto_gain(False, auto_gain_db)

if auto_rgb_gain_db is None:
    print("[WARN] invalid WB readback; keep auto white balance enabled")
else:
    safe_set_auto_whitebal(False, auto_rgb_gain_db)

print("[6/7] capture absolute exposure series")
for index, requested_us in enumerate(EXPOSURE_VALUES_US):
    if not safe_set_auto_exposure(False, requested_us):
        results.append(
            "requested_us={} exposure setting unsupported".format(
                requested_us
            )
        )
        continue

    img = settle_camera(SETTLE_FRAMES)
    stat = img.get_statistics()
    actual_us = safe_read("get_exposure_us", -1)
    actual_gain_db = safe_read("get_gain_db", -1)

    filename = FILE_PREFIX + "{:02d}_req_{:05d}_act_{:05d}.jpg".format(
        index + 1,
        requested_us,
        actual_us
    )

    print("[INFO] requested_us:", requested_us,
          "actual_us:", actual_us,
          "gain_db:", actual_gain_db,
          "L_mean:", stat.l_mean())

    try:
        save_photo(img, path_join(OUTPUT_DIR, filename))
        results.append(
            "{} requested_us={} actual_us={} gain_db={} "
            "L_mean={} L_stdev={}".format(
                filename,
                requested_us,
                actual_us,
                actual_gain_db,
                stat.l_mean(),
                stat.l_stdev()
            )
        )
    except Exception as error:
        print("[ERROR] photo save failed:", filename, error)
        results.append("{} save failed: {}".format(filename, error))

print("[7/7] save results")
results_name = FILE_PREFIX + "results.txt"
save_results(path_join(OUTPUT_DIR, results_name), results)

print("")
print("===== EXPOSURE TEST DONE =====")
print("board output:", OUTPUT_DIR)
print("Windows: E:\\exposure_sweep or E:\\exp_*")

while True:
    time.sleep_ms(1000)
