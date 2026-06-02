#!/usr/bin/env python3
"""Print a realistic mock PlatformIO monitor log for the integrated robot app.

This is for report screenshots when the robot/Arduino is not physically
available. It mirrors the serial messages produced by src/robot_app.cpp, but it
does not communicate with any hardware.
"""

from __future__ import annotations

import argparse
import random
import sys
import time


DEMO_STEPS = [
    ("forward low", -350, 332, 2000),
    ("stop", 0, 0, 700),
    ("forward high", -600, 570, 2000),
    ("stop", 0, 0, 700),
    ("left turn", 600, 570, 1500),
    ("stop", 0, 0, 700),
    ("right turn", -600, -570, 1500),
    ("stop", 0, 0, 700),
    ("u-turn", 600, 570, 3000),
    ("stop", 0, 0, 700),
]

QTR_MIN = [78, 84, 91, 88, 95, 90, 86, 82, 79]
QTR_MAX = [820, 835, 850, 872, 890, 864, 841, 825, 812]


def println(line: str = "", delay: float = 0.0) -> None:
    print(line, flush=True)
    if delay > 0:
        time.sleep(delay)


def values_to_text(values: list[int]) -> str:
    return "[" + ",".join(str(v) for v in values) + "]"


def qtr_raw_for(step_index: int, tick: int) -> list[int]:
    values = []
    line_centre = (step_index * 2 + tick) % 9
    for i in range(9):
        distance = abs(i - line_centre)
        base = 160 + random.randint(-18, 18)
        line_boost = max(0, 560 - distance * 210)
        values.append(max(0, min(1000, base + line_boost + random.randint(-25, 25))))
    return values


def qtr_calibrated(raw: list[int]) -> list[int]:
    calibrated = []
    for value, low, high in zip(raw, QTR_MIN, QTR_MAX):
        if high <= low:
            calibrated.append(0)
            continue
        scaled = int((value - low) * 1000 / (high - low))
        calibrated.append(max(0, min(1000, scaled)))
    return calibrated


def qtr_status(step_index: int, tick: int, calibrated_ready: bool) -> str:
    raw = qtr_raw_for(step_index, tick)
    if not calibrated_ready:
        return f"qtr_calibrating raw={values_to_text(raw)}"

    cal = qtr_calibrated(raw)
    peak = max(cal)
    found = 1 if peak >= 650 else 0
    surface = ["B" if v >= 500 else "W" for v in cal]
    weighted_sum = sum(v * i * 1000 for i, v in enumerate(cal))
    total = sum(cal)
    line = int(weighted_sum / total) if total else -1
    return (
        f"qtr raw={values_to_text(raw)} "
        f"cal={values_to_text(cal)} "
        f"surface=[{','.join(surface)}] "
        f"peak={peak} found={found} line={line}"
    )


def distance_text(tick: int) -> str:
    # Oscillate through plausible distances, occasionally reporting a wall.
    pattern = [38.6, 35.9, 31.2, 27.8, 23.4, 19.6, 22.1, 28.7, 34.5]
    return f"{pattern[tick % len(pattern)]:.1f}"


def print_startup(delay: float) -> None:
    println("--- Terminal on COM5 | 115200 8-N-1", delay)
    println("--- Available filters and text transformations: colorize, debug, default", delay)
    println("--- Quit: Ctrl+C | Menu: Ctrl+T | Help: Ctrl+T followed by Ctrl+H", delay)
    println("", delay)
    println("[INFO ] term3_robot integrated trial demo starting", delay)
    println(
        "features top_rgb=1 mechanical_kill=1 revive=1 qtr=1 "
        "ultrasonic=1 seed_dispenser=1 rfid=1",
        delay,
    )
    println("QTR RC bypass: pins=D45,D46,D47,D48,D49,D50,D51,D52,D53", delay)
    println("QTR RC bypass: emitter=disabled", delay)
    println("QTR: move array across floor and strip for 5 seconds", delay)
    println("Seed dispenser ready. Serial commands: d=dispense, c=close", delay)
    println("Motoron: starting Wire1", delay)
    println("motoron_ready=1 error=0", delay)
    println("rfid_addr=0x28 present=1 firmware=0x15 firmware_after_init=0x15 ready=1", delay)
    name, left, right, duration = DEMO_STEPS[0]
    println(f"mode={name} step=1/10 motor={left},{right} duration_ms={duration}", delay)


def print_monitor(cycles: int, delay: float, no_header: bool) -> None:
    random.seed(7)
    if not no_header:
        print_startup(delay)

    calibrated_ready = False
    rfid_uid = "none"
    revive = 0
    planter_states = ["idle"]

    for tick in range(cycles):
        if tick == 4:
            println("QTR calibration complete", delay)
            println(
                f"qtr_min={values_to_text(QTR_MIN)} "
                f"qtr_max={values_to_text(QTR_MAX)}",
                delay,
            )
            calibrated_ready = True
            println("button_stop=0", delay)

        if tick == 8:
            rfid_uid = "04 A1 32 7B"
            println(f"RFID UID: {rfid_uid}", delay)

        if tick == 11:
            println("seed_dispenser=dispense_start", delay)
            planter_states = ["upper_open", "upper_closing", "lower_open", "lower_closing", "idle"]

        if tick == 17:
            println("seed_dispenser=dispense_done", delay)
            planter_states = ["idle"]

        if tick == 20:
            revive = 1
            println("revive_button=1", delay)
        elif tick == 22:
            revive = 0
            println("revive_button=0", delay)

        step_index = (max(0, tick - 4) // 3) % len(DEMO_STEPS)
        step_name, left, right, duration = DEMO_STEPS[step_index]
        if tick >= 4 and (tick - 4) % 3 == 0:
            println(
                f"mode={step_name} step={step_index + 1}/10 "
                f"motor={left},{right} duration_ms={duration}",
                delay,
            )

        stopped = 1 if tick < 4 else 0
        state = "stopped" if stopped else "running"
        qtr = qtr_status(step_index, tick, calibrated_ready)
        planter = planter_states[min(len(planter_states) - 1, max(0, tick - 11))]
        status = (
            f"state={state} button_stop={stopped} revive={revive} motoron=1 "
            f"mode={step_name} {qtr} planter={planter} "
            f"distance_cm={distance_text(tick)} "
            f"rfid_ready=1 rfid_uid={rfid_uid}"
        )
        println(status, delay)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Mock the Arduino robot_app PlatformIO monitor output."
    )
    parser.add_argument(
        "--cycles",
        type=int,
        default=36,
        help="number of status cycles to print; use a large number for long screenshots",
    )
    parser.add_argument(
        "--delay",
        type=float,
        default=0.35,
        help="delay between printed lines in seconds",
    )
    parser.add_argument(
        "--no-header",
        action="store_true",
        help="hide the PlatformIO monitor header lines",
    )
    args = parser.parse_args()

    try:
        print_monitor(args.cycles, args.delay, args.no_header)
    except KeyboardInterrupt:
        println("")
        println("--- mock monitor stopped ---")
        return 130
    return 0


if __name__ == "__main__":
    sys.exit(main())
