Import("env")

import time

from SCons.Script import ARGUMENTS  # pylint: disable=import-error
from platformio.device.finder import SerialPortFinder


board = env.BoardConfig()
board.update("upload.disable_flushing", True)
board.update("upload.use_1200bps_touch", False)
board.update("upload.wait_for_upload_port", False)


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


def touch_then_wait_for_dfu(target, source, env):  # pylint: disable=unused-argument
    safe_autodetect_upload_port(env)
    upload_port = env.subst("$UPLOAD_PORT")

    if not upload_port or upload_port == "$UPLOAD_PORT":
        print("No serial upload port found; assuming the GIGA is already in DFU mode.")
        time.sleep(1)
        return

    print("Forcing GIGA bootloader using 1200bps touch on %s" % upload_port)
    env.TouchSerialPort(upload_port, 1200)
    print("Waiting for Arduino GIGA DFU device...")
    time.sleep(3)


env.AddMethod(safe_autodetect_upload_port, "AutodetectUploadPort")
env.AddPreAction("upload", touch_then_wait_for_dfu)
