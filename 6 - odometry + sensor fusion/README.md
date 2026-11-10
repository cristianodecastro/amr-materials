# VMX/Titan 3-wheel localization starter

This project is a new, separate VMX-Pi C++ project. It combines the relevant
ideas from projects 1–3: Titan wheel encoders, a 120-degree three-wheel
holonomic odometry model, VMX sonar inputs, the VMX's internal navX IMU, and
SmartDashboard telemetry. It uses the 2020 WPILib/GradleRIO and Studica vendor
library versions already used by the reference projects.

## What runs

The robot waits for the start button. After a debounced press, it zeros the
IMU yaw, resets the three Titan encoders, initializes its pose from the start
pose constants, and continuously:

1. Reads wheel encoder deltas and estimates body translation and rotation from
   the three wheel speeds.
2. Predicts heading from the gyro rate and corrects it toward the AHRS yaw.
3. Uses the AHRS world-linear acceleration as a small, damped correction to
   short-term translational velocity. Wheel odometry remains the main velocity
   estimate because integrating acceleration alone drifts quickly.
4. Pings the four ultrasonic sensors one at a time to limit acoustic
   interference. A range update is accepted only when it is plausible for the
   configured rectangular arena; the range update can adjust both position and
   heading using the sensor's mounting offset and beam direction.
5. Publishes pose, uncertainty, encoder-derived velocity, yaw rate,
   accelerometer values, and the latest accepted sonar range to SmartDashboard.

Accelerometers cannot independently measure planar yaw. Here, yaw is guided by
the gyro rate and the VMX AHRS's fused yaw; sonar measurements add a geometric
heading correction when the known arena walls and sensor layout make heading
observable. The IMU values are also exposed on SmartDashboard.

The robot has no joystick or motion command in the requested component list,
so this localization starter deliberately keeps all three drive outputs at
zero. `Hardware::DriveRobotRelative()` is provided for a later autonomous or
operator-control layer. It is not called by the localization loop.

## Required measurements to replace

All lengths are in metres. The values below are labeled placeholders in
`src/main/include/Constants.h`; replace them before relying on metric accuracy.

| Constant | Measurement needed |
|---|---|
| `WHEEL_RADIUS_M` | Effective rolling radius of each wheel |
| `WHEEL_CENTER_RADIUS_M[0..2]` | Distance from robot centre to each wheel contact point |
| `WHEEL_ANGLE_DEG` | Actual angular locations of each wheel around the centre, measured from robot-forward (+X) |
| `ARENA_WIDTH_M`, `ARENA_HEIGHT_M` | Interior dimensions of the rectangular arena and the wall coordinate frame |
| `START_X_M`, `START_Y_M`, `START_HEADING_DEG` | Robot pose when the start button is pressed |
| `REAR_SENSOR_X_M`, `REAR_SENSOR_HALF_SPACING_M`, `SIDE_SENSOR_Y_M` | Each sonar's sensing-face location relative to the robot centre |
| `SONARS[].beamAngleDeg` | Actual beam direction of each sensor relative to robot-forward |
| `ENCODER_COUNTS_PER_OUTPUT_REV` | Confirm the encoder count convention/configuration used by the Titan for your Maverick 75001 |
| `WHEEL_ENCODER_SIGN`, `WHEEL_MOTOR_SIGN` | Sign corrections after checking forward wheel direction and encoder count direction |
| `MAX_WHEEL_SPEED_MPS` | Safe calibrated wheel speed for mapping velocity to motor output |

The code assumes three omni wheels with their rolling directions tangent to a
circle around the robot, evenly spaced at 120 degrees. If the wheels or rollers
are arranged differently, the kinematic equations and wheel-angle table need
to match the actual chassis.

The arena correction assumes four sonar beams can measure the arena's
axis-aligned boundary walls. It rejects invalid ranges and residuals greater
than 0.40 m. Sonar returns from people, robot parts, or movable obstacles must
not be treated as arena-wall measurements; the arena/map model may need to be
expanded if those objects are in the sensors' beams.

## Wiring map

Use **WPILib logical DIO channel numbers** from the code. Studica documents
that these channel numbers map to VMX connectors and are affected by the
High-Current DIO direction jumper. These connections use the FlexDIO groups;
leave the High-Current DIO jumper at its default setting.

| Part | Code channel / port | VMX/Titan connector | Connection |
|---|---:|---|---|
| Rear-left ultrasonic | trigger DIO 0, echo DIO 1 | FlexDIO connector, channels 0–1 | Sensor signal A to trigger 0; signal B to echo 1; connect sensor power and ground |
| Rear-right ultrasonic | trigger DIO 2, echo DIO 3 | FlexDIO connector, channels 2–3 | Signal A to trigger 2; signal B to echo 3; connect power and ground |
| Left ultrasonic | trigger DIO 4, echo DIO 5 | FlexDIO connector, channels 4–5 | Signal A to trigger 4; signal B to echo 5; connect power and ground |
| Right ultrasonic | trigger DIO 6, echo DIO 7 | FlexDIO connector, channels 6–7 | Signal A to trigger 6; signal B to echo 7; connect power and ground |
| Start button | DIO 8 | FlexDIO header, first signal channel (8) | Normally-open momentary switch to ground; code defaults to active-low. Add a pull-up if the button/VMX input wiring does not provide one. |
| Maverick wheel 0 + encoder | Titan M0 | Titan Quad | Motor to M0; encoder cable to matching M0 encoder connection |
| Maverick wheel 1 + encoder | Titan M1 | Titan Quad | Motor to M1; encoder cable to matching M1 encoder connection |
| Maverick wheel 2 + encoder | Titan M2 | Titan Quad | Motor to M2; encoder cable to matching M2 encoder connection |
| VMX to Titan | CANH/CANL | VMX CAN ↔ Titan CAN | Connect CAN high to CAN high and CAN low to CAN low; code uses Titan CAN ID 42 |
| IMU | Internal VMX AHRS via `SPI::kMXP` | On-board | No external IMU cable; mount the VMX rigidly in the robot |

The four Studica ultrasonic sensors use one 4-pin FlexDIO connector each. Set
the applicable VMX sensor-power jumper to **5 V** for the Studica ultrasonic
sensor, and check the sensor's signal/power pinout before applying power. The
VMX has selectable 3.3 V/5 V I/O power; its documentation warns to check
voltage tolerance before connecting external devices.

The VMX WPI channel map starts with FlexDIO connectors on channels 0–7, then
the FlexDIO header on channels 8–11. The code's sonar pair assignments match
Studica's documented trigger/echo pairs: 0/1, 2/3, 4/5, and 6/7. The channel
numbers are not Titan motor numbers; Titan M0–M2 are a separate CAN-controlled
motor/encoder interface.

## Start and monitoring

- Keep the robot still during VMX IMU startup calibration.
- Align robot-forward with the arena +X direction when pressing start, because
  the code zeros AHRS yaw at that moment.
- SmartDashboard keys include `pose_x_m`, `pose_y_m`, `pose_heading_deg`,
  `pose_sigma_x_m`, `pose_sigma_y_m`, `pose_sigma_heading_deg`,
  `imu_yaw_rate_dps`, world-linear acceleration X/Y, all three raw
  accelerometer axes, all three raw gyroscope axes, and the most recent
  accepted sonar range. Each sonar also publishes its latest value
  as `sonar_back_left_m` (and corresponding left/right/back-right keys), a
  validity flag, and whether it contributed to a pose correction.
- The active button polarity is controlled by
  `START_BUTTON_ACTIVE_LOW` in `Constants.h`.

## References

- [Studica VMX connector setup](https://docs.wsr.studica.com/en/latest/docs/VMX/setup.html)
- [Studica WPI channel addressing](https://docs.wsr.studica.com/en/latest/docs/VMX/wpi-channel-addressing.html)
- [Studica ultrasonic sensor API and valid DIO pairs](https://docs.wsr.studica.com/en/latest/docs/Sensors/ultrasonic.html)
- [Studica VMX navX/IMU guide](https://docs.wsr.studica.com/en/latest/docs/VMX/navx.html)
- [Studica Titan Quad guide](https://docs.wsr.studica.com/en/latest/docs/Titan/titan.html)
- [Studica Maverick 75001 motor specifications](https://www.studica.com/maverick-12v-dc-gear-motor-with-encoder)
