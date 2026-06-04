
# Final Viva/Test Run Log

## Test Information

**Date:** 5 June
**Code version:** `Jason_combined_code/`
**Main file:** `Jason_combined_code/Main.ino`
**Purpose:** Final integrated robot test for the viva/test run.

---

## 1. Startup and Calibration Test

### What was tested

The robot startup sequence, including IR line sensor calibration and IMU gyro calibration.

### How it was tested

The robot was powered on and the final code was uploaded from `Jason_combined_code/`. During startup, the line sensor array was moved over the floor and black line for calibration. The robot was then placed flat and still while the IMU gyro bias calibration was completed.

### What worked

* The robot entered the calibration sequence correctly.
* The IR sensors produced calibrated values.
* The IMU gyro calibration completed after the robot was kept still.
* Serial Monitor printed calibration and ready messages.

### What did not work / limitations

* The calibration quality depended on moving the sensors across both the floor and the line properly.
* If the robot was moved during IMU calibration, the turn accuracy became worse.

### Evidence

* Serial Monitor output showing calibration messages.
* Photo/screenshot of Serial Monitor.
* Short video of startup calibration if available.

---

## 2. Line Following Test

### What was tested

The 9-channel line sensor reading, line detection, line position estimation, and proportional motor correction.

### How it was tested

The robot was placed on the black line after calibration. The robot followed the track using the line-following control loop.

### What worked

* The robot detected the black line.
* The robot adjusted left and right motor speeds based on line position.
* The robot could continue moving along straight sections of the line.

### What did not work / limitations

* Line following became less stable when the lighting changed or when the sensor height changed.
* Sharp junctions required careful calibration and sometimes caused unstable behaviour.

### Evidence

* Video of the robot following the line.
* Serial Monitor output showing line position, error, and motor commands.

---

## 3. IMU Turning Test

### What was tested

IMU-based 90-degree and 180-degree turning using gyro Z-axis integration.

### How it was tested

The robot was commanded to turn left/right using the turning functions. The robot used gyro angle integration and attempted to reacquire the line after turning.

### What worked

* The robot could rotate using IMU angle feedback.
* The turn functions stopped the motors after the target angle or line detection.
* The robot could reacquire the line after some turns.

### What did not work / limitations

* Turning accuracy depended on successful gyro calibration.
* IMU drift and vibration affected the final angle.
* Some turns required retuning of target angles because the physical robot overshot slightly.

### Evidence

* Serial Monitor output showing turn angle.
* Video of 90-degree and/or 180-degree turns.

---

## 4. RFID and Server Test

### What was tested

RFID tag detection and server communication using the serial command `9`.

### How it was tested

An RFID tag was placed near the reader. The serial command `9` was sent through Serial Monitor. The robot scanned the RFID tag, sent an `isFertile` request to the server, and printed the reply.

### What worked

* The RFID reader detected tag UIDs.
* The robot sent the tag information to the server.
* The robot printed the UID, fertility state, and x/y position when a reply was received.

### What did not work / limitations

* RFID detection depended on tag distance and orientation.
* Server replies sometimes depended on WiFi/MQTT connection quality.
* If the tag was moved too quickly, the RFID reader sometimes missed it.

### Evidence

* Screenshot of Serial Monitor showing RFID UID and server reply.
* Photo/video of RFID test.

---

## 5. Obstacle Avoidance Test

### What was tested

The scripted obstacle avoidance sequence using the forward ultrasonic sensor, RFID junction detection, line following, and IMU turning.

### How it was tested

The serial command `8` was used to reset and select the obstacle avoidance sequence. An obstacle was placed in front of the robot on the test path.

### What worked

* The forward ultrasonic sensor detected an obstacle.
* The robot entered the obstacle avoidance state machine.
* The robot performed a scripted sequence of turns and straight movements.

### What did not work / limitations

* The sequence depended on the obstacle and junction layout matching the tested route.
* Ultrasonic readings could fluctuate if the obstacle surface was angled.
* If RFID junction detection failed, the sequence could stop or become inaccurate.

### Evidence

* Video of the obstacle avoidance attempt.
* Serial Monitor output showing obstacle avoidance state changes.

---

## 6. Tunnel / Wall Following Test

### What was tested

Tunnel entry, side ultrasonic wall following, gate detection, and exit gate handling.

### How it was tested

The robot was placed near the tunnel entrance. The forward ultrasonic sensor was used to detect gates, and the side ultrasonic sensor was used for wall-following correction.

### What worked

* The robot could detect the base gate using the forward ultrasonic sensor.
* The robot could use side distance for wall following.
* The motor speeds changed based on side distance error.

### What did not work / limitations

* Wall following depended on the side wall being on the expected side.
* Ultrasonic readings could be noisy near corners or angled surfaces.
* Thresholds required tuning for the real arena setup.

### Evidence

* Video of tunnel/wall-following test.
* Serial Monitor output showing gate/wall-following messages.

---

## 7. Seed Dispenser Test

### What was tested

The upper and lower servo sequence for dispensing one seed.

### How it was tested

The seed dispenser was triggered using the planting function. The upper servo opened and closed first, then the lower servo opened and closed.

### What worked

* The upper and lower servo gates moved in the expected sequence.
* The dispenser could release a seed when the mechanism was aligned correctly.

### What did not work / limitations

* The result depended on seed size and mechanical friction.
* Servo angles needed manual adjustment to avoid jamming.
* The manually generated servo pulses must continue to be serviced during longer code sections.

### Evidence

* Short video of the servo/seed dispenser test.
* Photo of the dispenser mechanism.

---

## Final Notes

The `Jason_combined_code/` folder was used as the final integrated code version. The main tested behaviours included startup calibration, line following, IMU turning, RFID/server testing, obstacle avoidance, tunnel/wall following, and seed dispensing.

Some behaviours worked reliably in controlled tests, while others depended on environmental conditions such as lighting, sensor alignment, WiFi/server connection, RFID tag position, and ultrasonic sensor noise.

The final code is suitable as the viva/test run version, but some behaviours may still require manual setup, calibration, or reset during physical testing.
