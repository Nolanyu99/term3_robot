Import("env")

import os
import subprocess
import time

from SCons.Script import ARGUMENTS  # pylint: disable=import-error
from platformio.device.finder import SerialPortFinder


board = env.BoardConfig()
board.update("upload.disable_flushing", True)
board.update("upload.use_1200bps_touch", False)
board.update("upload.wait_for_upload_port", False)

DFU_WAIT_SECONDS = 30
DFU_IDS = ("2341:0366", "2341:035b", "2341:035f", "2341:0360")


def dfu_tool_path(env):
    platform = env.PioPlatform()
    package_dir = platform.get_package_dir("tool-dfuutil-arduino")
    if package_dir:
        tool_name = "dfu-util.exe" if os.name == "nt" else "dfu-util"
        return os.path.join(package_dir, tool_name)
    return "dfu-util"


def has_dfu_device(env):
    try:
        result = subprocess.run(
            [dfu_tool_path(env), "-l"],
            check=False,
            capture_output=True,
            text=True,
            timeout=5,
        )
    except Exception:
        return False

    output = (result.stdout or "") + (result.stderr or "")
    return any(device_id.lower() in output.lower() for device_id in DFU_IDS)


def wait_for_dfu_device(env, timeout_s):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if has_dfu_device(env):
            print("Arduino GIGA DFU device detected.")
            return True
        time.sleep(0.5)
    return False


def safe_autodetect_upload_port(env):
    initial_port = env.subst("$UPLOAD_PORT")
    if initial_port == "$UPLOAD_PORT":
        initial_port = None

    upload_port = SerialPortFinder(
        board_config=env.BoardConfig(),
        upload_protocol=env.subst("$UPLOAD_PROTOCOL"),
        verbose=int(ARGUMENTS.get("PIOVERBOSE", 0)),
    ).find(initial_port)

    if upload_port:
        env.Replace(UPLOAD_PORT=upload_port)
        print("Auto-detected: %s" % upload_port)
        return

    print("No serial upload port found; continuing because dfu-util can find the DFU device directly.")


def wait_for_manual_dfu(target, source, env):  # pylint: disable=unused-argument
    if has_dfu_device(env):
        print("Arduino GIGA DFU device detected.")
        return

    print("Arduino GIGA DFU device not detected.")
    print("Double-tap RESET now; upload will continue when the BOOT0 LED is green.")
    if not wait_for_dfu_device(env, DFU_WAIT_SECONDS):
        print("No DFU device detected. Double-tap RESET until the BOOT0 LED is green, then upload again.")
        env.Exit(1)


env.AddMethod(safe_autodetect_upload_port, "AutodetectUploadPort")
env.AddPreAction("upload", wait_for_manual_dfu)
