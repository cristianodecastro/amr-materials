# Project 7: robot software, decisions, and operating notes

This document records what was designed and implemented during the project
work in this chat, why the design changed over time, how the VMX, Titan,
sensors, controller, and dashboards are connected, what has actually been
observed on the robot, and what remains to be calibrated. It describes the
current source tree in `7 - SkillsChallenge/SkillsChallenge`.

## 1. Project goal and boundaries

The original goal was to understand the earlier Studica robotics examples
(`1 - mobility + sensor drive`, `2 - sensor drive + set position`, and
`3 - oms + gyro`) and combine the relevant drive, position, and sensor-fusion
ideas into a mobile-robot project. Localization code was then developed in
project 6 and transferred to project 7.

Project 7 is the independent deliverable. Its generated SkillsChallenge
configuration is the project's golden template. The Gradle wrapper, GradleRIO
version, target configuration, and vendor-library declarations are intended
to remain those of project 7. Source code and project documentation can
change; project 7 does not import, link against, or need projects 1 through 6
to build or run. The machine still needs the appropriate WPILib 2020 Java
runtime, Raspberry Pi cross compiler, and cached WPILib/Studica/navX
dependencies. These installed tools are not dependencies on the example
projects.

The software has three source-selected operating paths:

1. **Manual teleoperation** is the current default. The driver's left stick
   commands robot-relative translation in any direction, including diagonals.
2. **Forward 1000 mm exercise** is an encoder-only test started by the
   physical button. It is available by changing source constants.
3. **Localization evaluation** reads the IMU, wheel encoders, and sonars to
   estimate and correct arena pose. It is also available by changing source
   constants; it is not currently active while manual teleoperation is
   enabled.

These are mutually prioritized code paths, not three simultaneously running
controllers. `TELEOP_DRIVE_ENABLED` takes priority; otherwise
`FORWARD_TEST_ENABLED` selects the one-metre exercise; otherwise the source
runs the stationary localization evaluation mode.

### Earlier projects used as references

The old projects are separate, earlier programs, not libraries called by
project 7. Their source helped identify the existing hardware conventions and
drive/sensing ideas. They were written for their own robots/configurations;
their channel maps, dimensions, units, wheel labels, and blocking control loops
must not be copied blindly into the new robot.

| Reference project | What its source does | Relationship to project 7 |
|---|---|---|
| `1 - mobility + sensor drive` | `Hardware` constructs a Titan Quad at CAN 42, drive motors on ports 1/0/2 (called left/back/right), corresponding Titan encoders, navX on SPI MXP, start/stop buttons, analog Sharp sensors, and a Studica Cobra sensor. `Movement::PositionDriver` samples encoder changes at a fixed 0.2 s interval, converts to wheel speed, solves three-wheel forward kinematics, integrates X/Y and encoder-derived heading, calculates bounded proportional movement toward a requested pose, and uses inverse kinematics to set motor PWM. It also has a blocking `sensor_drive(dist)` loop using the right distance sensor and a stop-button check. Dashboard keys expose pose, velocities, desired wheel speeds, and button status. | Reference for the overall three-wheel odometry/drive cycle and Titan/navX access. Its code uses centimetres, fixed timing, old wheel labels/angles, and its own PWM scales. Project 7 uses mm, measured loop time, the new M0/M1/M2 geometry, and scheduled nonblocking periodic updates. |
| `2 - sensor drive + set position` | Keeps the first project's encoder odometry and adds a start-button sequence, starting pose/heading setup, left/right ultrasonic ranging, a front Sharp distance sensor, and Cobra line alignment. Its `sensor_drive(dist, direction)` switches among front/left/right distance routines and repeatedly drives until a target distance tolerance is reached. The sample robot sequence sets/revises pose from wall distances and aligns against a line. | Reference for sensor-guided travel, setting pose from arena references, and line alignment. Project 7 instead gives four sonars separate trigger/echo pairs, uses finite-range and wall-prediction checks, and applies a covariance pose correction. It does not currently run a blocking sensor-drive loop. |
| `3 - oms + gyro` | Extends the previous movement/hardware structure with gyro-based heading: it reads navX yaw, applies a configured offset, wraps the result, and uses it for heading error and field-to-robot velocity conversion. It adds OMS elevator/gripper routines driven by Titan port 3 and an encoder, limit switches, and a servo; the example robot starts an OMS height move. | Reference for using IMU yaw to stabilize heading and for the legacy mechanism code. Project 7 uses gyro rate to predict yaw and AHRS yaw to correct it, plus an encoder yaw estimate. The current requested robot has three drive motors and no OMS/elevator/servo/limit-switch subsystem, so those outputs were not added. |

All three examples construct Studica Titan motor/encoder objects using the
configured Titan CAN ID; they access the navX through the VMX SPI MXP interface
and publish values to SmartDashboard. Projects 1 through 3 use blocking
`delay()`-based loops for movement and sensor routines. Project 7 retains the
useful high-level ideas—wheel kinematics, encoder pose tracking, navX yaw,
wall ranges, and dashboard telemetry—but organizes them behind WPILib
hardware access, a command-scheduler subsystem, explicit mode selection,
unit-labelled telemetry, and separate source constants for the new hardware.
Project 7 has its own current I/O table in section 3; for example, its four
sonars occupy DIO 0..7 and its start button is DIO 8, whereas the older
projects have different assignments and fewer ultrasonic sensors.

## 2. Robot hardware and the agreed layout

The robot configuration discussed in the project consists of:

- A Studica VMX controller running the WPILib/GradleRIO 2020 robot program.
- A Studica Titan Quad motor controller on the CAN network, configured in
  source as CAN ID 42.
- Three Maverick 75001 motor/encoder assemblies on Titan outputs M0, M1, and
  M2. The encoders are read from the Titan; they do not consume VMX DIO pins.
- Three omni wheels spaced 120 degrees apart. The supplied top-view layout
  places M0 at 150 degrees (quadrant II), M1 at 30 degrees (quadrant I), and
  M2 at 270 degrees (toward the quadrant III/IV boundary).
- Four ultrasonic sensors: two at the rear, one on each side.
- A physical, active-low start button on VMX DIO 8.
- The VMX's onboard navX IMU, accessed through SPI MXP by the Studica/WPILib
  software interface. There is no external IMU wire in this assignment.
- The Studica USB/Bluetooth remote paired to the Windows Control Station PC.
  Control Station forwards joystick input to the robot over its Driver
  Station connection; the current program does not pair directly with the
  controller on the VMX.

The user confirmed a **102 mm wheel diameter** (51 mm radius), a **168 mm
wheel-center radius** for all three wheels, the Titan-port-to-wheel mapping
above, and robot-forward as +X in the originally supplied layout description.
The wheel radius and center radius are used for different calculations:
wheel radius converts encoder rotations into surface distance; wheel-center
radius converts robot yaw rate into wheel rolling speed. Changing the
168 mm center radius cannot, by itself, change a zero-yaw translation command.

The physical forward-axis interpretation is still being reconciled with the
arena observation described in section 8. Treat the empirical -90-degree
drive correction as awaiting a repeatable robot check; do not regard it as a
measured chassis-to-IMU calibration.

## 3. VMX and Titan connections

The source assigns the following logical channels. WPI channel numbers are
software channel indices; use the VMX connector documentation and its jumper
configuration to identify the matching physical pins. The sensors discussed
here were connected to FlexDIO with a 5 V sensor supply.

| Device | WPI/source assignment | Physical connection / notes |
|---|---|---|
| Rear-left ultrasonic | Trigger DIO 0, echo DIO 1 | FlexDIO pair 0/1; sensor signal A is the trigger and signal B is the echo in the wiring plan used during this project. |
| Rear-right ultrasonic | Trigger DIO 2, echo DIO 3 | FlexDIO pair 2/3; A trigger, B echo. |
| Left ultrasonic | Trigger DIO 4, echo DIO 5 | FlexDIO pair 4/5; A trigger, B echo. |
| Right ultrasonic | Trigger DIO 6, echo DIO 7 | FlexDIO pair 6/7; A trigger, B echo. |
| Start button | DIO 8, active low | Normally-open button between the DIO signal and ground. The code inverts the raw input to report pressed=true. |
| M0 motor and encoder | Titan CAN 42, output/encoder 0 | Wheel in quadrant II at 150 degrees. |
| M1 motor and encoder | Titan CAN 42, output/encoder 1 | Wheel in quadrant I at 30 degrees. |
| M2 motor and encoder | Titan CAN 42, output/encoder 2 | Wheel at 270 degrees. |
| VMX to Titan | CANH and CANL | CANH-to-CANH and CANL-to-CANL; Titan address is configured as 42. |
| navX IMU | VMX `SPI::kMXP` | Onboard; keep the board securely mounted and aligned with the chosen body axes. |
| Remote controller | Control Station joystick port 0 by default | Controller goes to the Windows PC over USB or Bluetooth; Control Station forwards joystick data to the robot. |

The ultrasonic sensors require their specified 5 V and ground connections
as well as signal wiring. A powered sensor is not necessarily being triggered:
in the VMX HAL version used by this project, the FlexDIO watchdog can block
trigger outputs while the robot is Disabled. Enabling the robot through
Control Station is required for the diagnostic localization mode to generate
sonar trigger pulses. The sonar diagnostics deliberately report this disabled
condition rather than treating 5 V power as proof of operation.

Studica documents the VMX [connector blocks and sensor wiring](https://docs.wsr.studica.com/en/latest/docs/VMX/setup.html),
[WPI channel addressing](https://docs.wsr.studica.com/en/latest/docs/VMX/wpi-channel-addressing.html),
[navX orientation and calibration](https://docs.wsr.studica.com/en/latest/docs/VMX/navx.html),
and [Titan motor control](https://docs.wsr.studica.com/en/latest/docs/Titan/titan.html).
The earlier project work also referenced Studica's
[WPILib project creation](https://docs.dev.studica.com/en/latest/docs/WPILib/creating-a-project.html)
and [vendor library setup](https://docs.dev.studica.com/en/latest/docs/WPILib/adding-vendor-libs.html)
instructions when preserving the project 7 template.

## 4. Current mode: left-stick teleoperation

`src/main/include/LocalizationConstants.h` currently sets
`TELEOP_DRIVE_ENABLED = true`. The scheduler calls the localization subsystem
periodically, and its `Periodic()` method dispatches directly to the teleop
handler before the forward-test or localization paths.

### Input handling

The current defaults expect the first Driver Station joystick (port 0), with
the left stick on axis 0 (X) and axis 1 (Y):

| Stick input | Source interpretation | Desired robot movement |
|---|---|---|
| X positive | Right | Right |
| X negative | Left | Left |
| Y negative | Forward | Forward |
| Y positive | Backward | Backward |
| X and Y together | Both translation components | Diagonal |
| Both near zero | Neutral | Stop |

This axis numbering is a common gamepad assumption, not a mapping confirmed
from the particular controller's USB and Bluetooth modes. The Studica product
page confirms that the remote supports USB and Bluetooth, but does not specify
the host operating system's axis indices. The **Teleop** dashboard exposes
the raw X/Y values before the robot is enabled. Use Windows `joy.cpl` and the
dashboard to verify the actual mode. Change `DRIVER_JOYSTICK_PORT`,
`DRIVER_LEFT_X_AXIS`, `DRIVER_LEFT_Y_AXIS`, `DRIVER_LEFT_X_SIGN`, or
`DRIVER_LEFT_Y_SIGN` in `LocalizationConstants.h` if the readings differ.
Disconnect other gamepads during initial setup if they shift which device is
joystick port 0.

### Translation shaping and output

- A circular 12% center deadband suppresses stick noise. Values outside the
  deadband are rescaled smoothly to the requested speed range.
- The vector magnitude is capped, so a full diagonal does not request
  sqrt(2) times the maximum straight-line speed.
- The full-stick nominal request is 250 mm/s. A 500 mm/s^2 limit smooths
  changes in commanded vector. Centering the stick requests zero immediately.
- The drive output is capped at 25%. `Hardware::DriveRobotRelative()` maps
  the desired translation through the three-wheel inverse kinematics, and
  `DriveWheelsMMps()` scales the wheel vector together to preserve its ratios.
- No yaw rate is commanded. The mode does not implement heading hold,
  field-oriented driving, obstacle avoidance, sonar correction, or a measured
  velocity PID loop. `MAX_WHEEL_SPEED_MMPS = 1000` is still a placeholder, so
  mm/s dashboard values are requested speeds rather than verified physical
  speeds.
- The physical Start button is unused in this mode. Wheel encoder counts are
  displayed for observation, but they do not adjust the teleop command.

After Control Station attaches, the controller must report both configured
axes. A missing Driver Station or missing axes commands zero and disarms the
stick. Disabled, Autonomous, or Test mode also commands zero and disarms it.
After enabling, changing modes, or reconnecting, center the stick until
**Stick armed** is true before moving it. This prevents a stick held off-center
through enable from immediately starting the robot. If a PC continues to send
stale axis values as valid controller input after a Bluetooth failure, the
robot cannot independently detect the physical radio failure; the status
reflects the data reported through Control Station.

### Teleop dashboard

Robot startup selects the **Teleop** Shuffleboard tab. Its 14 values are
arranged in three rows:

1. Drive status, Enabled, Controller, Stick armed.
2. Raw left-stick X/Y and requested forward/left speeds in mm/s.
3. M0/M1/M2 raw encoder counts and applied output percentages.

These are enough to check controller recognition, axis signs, drive request,
motor output, and encoder response. This compact mode does not run sonar
polling or publish the localization IMU view. Old widgets saved in the
Shuffleboard client can remain visible on older tabs; select **Teleop** for
the current layout. The code disables LiveWindow's generic telemetry in the
default mode so raw DIO Boolean widgets do not overwhelm the purpose-built
numeric dashboard.

Studica's [remote-controller page](https://www.studica.com/remote-controller-bluetooth)
describes USB/Bluetooth support. Its [Control Station instructions](https://docs.dev.studica.com/en/latest/docs/ControlStation/index.html)
describe joystick forwarding, the Joysticks indicator, connection, operation
mode, and enable/disable controls.

## 5. Optional encoder-only one-metre exercise

Set `TELEOP_DRIVE_ENABLED = false` and `FORWARD_TEST_ENABLED = true` to select
this mode. It uses the physical start button and the three Titan encoder
readings. It does not ping sonars, update localization, or require a ready IMU.
The robot must be enabled in Teleoperated mode. Start must first be released
to arm; a 60 ms debounced press snapshots the three current counts and begins
the one-metre run. A held press cannot retrigger the run. Disabling or leaving
Teleoperated mode stops the motors and aborts the run; enabling again does not
resume it.

`ForwardDriveController.h` is a hardware-independent controller used by the
subsystem. It:

1. Converts encoder count deltas into wheel surface millimetres using wheel
   circumference, output counts per revolution, external motor-to-wheel
   ratio, encoder sign, and the direction coefficient.
2. Estimates translation along the commanded direction with a least-squares
   projection: `sum(projected wheel travel * coefficient^2) / sum(coefficient^2)`.
   For the symmetric wheel arrangement, this suppresses some common wheel
   displacement caused by a small amount of rotation.
3. Ramps requested speed up, limits the nominal cruise request, and reduces
   requested speed near the goal. It stops at 1000 mm estimated travel.
4. Applies bounded wheel synchronization to the two active wheels so a
   lagging encoder receives more requested speed and a leading one receives
   less. This correction is bounded at 80 mm/s.
5. Stops on a 20 s total timeout, two seconds without progress on a driven
   wheel, more than 20 mm projected reverse travel, invalid encoder values,
   or more than 100 mm disagreement between wheel travel projections.

The one-metre dashboard shows run status, enabled/start/complete state,
travel/target/remaining/elapsed values, three encoder counts, and three
applied output percentages. Its status keeps the abort or fault message visible
after disabling.

## 6. Optional localization and sensor fusion

Set both `TELEOP_DRIVE_ENABLED = false` and `FORWARD_TEST_ENABLED = false` to
select the localization evaluation mode. This path holds all Titan motor
outputs at zero so sensors and odometry can be observed without driving. It
does not execute while teleop or the forward exercise is selected.

### Start and odometry flow

The program waits until the VMX IMU is connected and no longer calibrating.
After the active-low start button is held long enough to pass the localization
debounce (three periodic samples), it zeroes the navX yaw, records current
encoder baselines, and initializes pose from `START_X_MM`, `START_Y_MM`, and
`START_HEADING_DEG`.

On each update, `Localization::UpdateOdometry()`:

1. Reads the three encoder changes, applies each configured encoder sign, and
   converts counts to wheel speed in mm/s.
2. Builds the three-wheel tangent rolling model from wheel radial angles and
   center radii, then solves its 3x3 system for body X/Y velocity and
   encoder-derived yaw rate.
3. Predicts heading by blending VMX gyro yaw rate with encoder wheel yaw rate.
   It then corrects that prediction toward the navX AHRS yaw. Hardware access
   converts the navX clockwise-positive yaw and rate into the localization
   counterclockwise-positive convention.
4. Uses encoder-derived world velocity as the main translational estimate,
   blended with a small, damped integration of VMX world linear acceleration.
   The accelerometer contribution has a deadband and 0.10 blend weight to
   reduce the effect of bias and drift. Acceleration can help translational
   estimates but does not independently measure planar yaw.
5. Integrates velocity into arena X/Y and grows a 3-state covariance for X,
   Y, and heading.

This is a lightweight covariance-based sensor-fusion estimator; it is not a
full general-purpose SLAM system. It assumes the wheel geometry, encoder
signs, IMU-to-body alignment, and starting pose are calibrated.

### Ultrasonic polling and pose correction

The four sonars are polled sequentially. WPILib automatic ultrasonic mode is
disabled so there is only one ping scheduler. `Ping()` requests a short active
trigger pulse and resets that sensor's echo capture. The code waits at least
60 ms before reading, then leaves a 40 ms interval before the next sensor.
This spacing limits cross-talk and allows time for the longest configured
range and VMX/HAL update. A complete four-sensor cycle is roughly 400 ms at
the normal loop cadence.

Usable ranges are 20 through 4000 mm. `Localization::PredictSonarRange()`
projects the configured sensor face and beam direction to the nearest valid
wall in the rectangular arena. A reading is rejected if invalid, outside the
range, if no wall can be predicted, or if its residual exceeds 400 mm. For an
accepted range, a numerical Jacobian and scalar covariance correction update
pose and heading. A person or object in the beam can resemble a wall, and bad
arena/sensor geometry can make a correct sensor reading produce an incorrect
pose correction.

The sonar mount coordinates and arena dimensions are currently placeholders.
Current defaults assume a 2000 x 2000 mm arena, a 500 x 250 mm start position,
rear sensor X=-100 mm and half-spacing 80 mm, and side sensor Y offsets of
120 mm. The configured rear beams point at 180 degrees; left/right beams point
at +90/-90 degrees. Measure the sensor faces, beam directions, arena walls,
origin, and initial pose before relying on corrections.

### Localization dashboard and sonar debugging history

The compact **Robot** tab shows readiness, enabled/IMU/start status, count of
valid sonar sensors, four numeric ranges in mm, arena X/Y, fused heading,
body velocities, yaw rate, encoder counts, yaw, and world linear acceleration.
Detailed diagnostics are off by default. Set
`ENABLE_DETAILED_TELEMETRY = true` for ping requests, echo responses,
timeouts, sonar status/ranges, raw IMU axes, and localization uncertainty.

Earlier, sonar entries appeared as Boolean values because generic LiveWindow
DIO widgets describe digital pin states, not a numeric distance. The useful
distance is obtained from WPILib's ultrasonic range API and is published as a
number in millimetres; the compact Robot dashboard replaced the confusing
generic view. The user checked that the sonar hardware had 5 V and that signal
A/B were connected as planned. Four sensors initially appeared unavailable,
then sensor values were observed; later the left sensor alone was reported
not working. The source gives the left sonar its own configured DIO 4/5 pair
but runs it through the same polling and validation path as the other sensors.
No distinct missing left-sensor code path was found, and no confirmed root
cause for the intermittent left reading was established in this chat.

The VMX disabled-state IO watchdog explained why trigger pulses/ranges could
be absent even when a sensor had 5 V. Diagnostics therefore need the robot
enabled. A `_ping_requests` counter proves that software called `Ping()`; it
does not electrically prove a pulse reached the connector. A numeric `-1`
denotes unavailable/invalid in the compact view; detailed status separates
waiting, disabled, no echo, out-of-range, and valid results. Boolean `_valid`
fields are intentionally status indicators, not distances.

## 7. One-metre arena attempt and direction correction

The user requested a one-metre forward test without using sensors. The first
arena attempt physically translated about **535 mm to the left**. The final
Shuffleboard screenshot showed:

| Dashboard value | Screenshot reading |
|---|---:|
| Travel | 525.972 mm |
| Target | 1000 mm |
| Remaining | 474.028 mm |
| Elapsed | 5.200 s |
| Complete | false |
| M0/M1/M2 raw counts | -3859 / -3178 / 7325 |
| M0/M1/M2 output at screenshot | 0 / 0 / 0% |

The screenshot was captured after the robot was disabled. The run was
incomplete and the measured physical displacement closely matched the
displayed encoder travel. The absolute counts do not provide a calibrated
travel scale without the per-run baseline. There was no evidence to support
scaling the encoder distance to force 535 mm to equal 1000 mm. The old
disabled status did not reveal why the run had ended; status retention after
disable was added later.

The next source revision applied `FORWARD_TEST_DIRECTION_DEG = -90` to rotate
the requested vector clockwise relative to the originally interpreted +X
wheel-layout direction. Commands and encoder projections use the same derived
wheel coefficients. For this direction the idealized pattern is M0 positive,
M1 negative, M2 zero. The recent teleop mode reuses that same correction for
its robot-forward axis. This is an empirical hypothesis based on the observed
leftward translation, not a measured frame alignment; the robot has not yet
reported a confirming run after that change. Substantial rotation during the
first attempt, an incorrect motor sign, a different actual wheel rolling axis,
or a layout/IMU convention mismatch would require revisiting it.

## 8. Calibration values and unresolved measurements

| Value | Current source value | Status |
|---|---:|---|
| Wheel diameter | 102 mm | User specified; radius is 51 mm. |
| Wheel-center radius | 168 mm for each wheel | User changed the earlier 150 mm placeholder to 168 mm. |
| Radial wheel angles | M0 150°, M1 30°, M2 270° | Supplied by user in a top-view trigonometric-quadrant description. |
| Titan output counts/revolution | 1464 | Provisional; confirm against actual Titan/75001 encoder count convention. |
| Motor-to-wheel ratio | 1.0 | Assumes direct drive; verify external gearing. |
| Encoder and motor signs | all +1.0 | Must agree with installed motor and encoder wiring; the first drive observation makes this worth rechecking. |
| Maximum wheel surface speed | 1000 mm/s | Placeholder used to map mm/s requests to normalized output. |
| Arena size | 2000 x 2000 mm | Placeholder. |
| Starting pose | (500, 250) mm, 0° | Placeholder. |
| Sonar mount faces | rear X=-100 mm, rear half-spacing 80 mm, side Y=120 mm | Symmetric placeholders; measure actual sensing faces from robot center. |
| Sonar beam directions | rear 180°, left +90°, right -90° | Geometric assumptions; verify actual sensor orientation and names. |
| Joystick mapping | port 0, axes 0/1, signs +1/-1 | Common default; verify in Windows `joy.cpl` and the Teleop raw-axis cards for the selected USB/Bluetooth mode. |
| Manual drive frame correction | -90° relative to source wheel-layout frame | Empirical, based on the first one-metre attempt moving left; not yet confirmed by a subsequent physical run. |

The one-metre estimate uses the provisional scale
`2*pi*51 / 1464 = 0.218881455 mm/count`, then divides each signed wheel
travel by its projection coefficient. Any count-per-revolution or gearing
error affects the reported travel. Wheel slip, wheel compression/effective
diameter, battery voltage, surface friction, coast distance, and the open-loop
output/speed scale also affect physical performance.

## 9. Build, deployment, and earlier setup errors

Open `7 - SkillsChallenge/SkillsChallenge` as the project root. On the
development machine, the commands that have been used are:

```powershell
$env:JAVA_HOME = 'C:/Users/Public/wpilib/2020/jdk'
.\gradlew.bat assemble -PteamNumber=23 --offline --console=plain
.\gradlew.bat deploy -PteamNumber=23 --offline
```

The project previously built and deployed successfully after dependencies
were cached. A past deploy attempt failed because `--offline` could not find
the 2020 RoboRIO/VMX JRE artifact in Gradle's cache. Another attempt failed
because `JAVA_HOME`/`java` was not available to the shell. The later `downloadAll`
attempt reached Maven but failed on an unavailable
`cameraserver-cpp-2020.3.2-linuxaarch64xenialdebug.zip` artifact. The reported
failure concerned that legacy camera/native dependency, not the source-level
joystick code. The project was then deployed successfully. The project 7
Gradle and vendor files were not altered to remove that missing platform
artifact. The current teleop source last passed the project's offline debug
and release `assemble` build; it has not been physically tested on the robot.

Control Station connects to the robot at `10.0.23.2` for team 23 in the
configured Wi-Fi setup. Studica's Control Station can also use the VMX Ethernet
address when a wired network connection is configured. Selecting Teleoperated
and enabling/disabling are still required; merely connecting the remote to
Windows does not enable the robot. The Joysticks status should show that a
controller is being forwarded.

## 10. How the work evolved

1. The request began with a review of existing Studica projects and a new
   three-wheel robot design combining odometry, four ultrasonic sensors,
   start button input, and the VMX IMU.
2. Project 7 was identified as the golden template. Source from the prior
   odometry/sensor-fusion work was moved into it while guarding its generated
   project configuration. All geometry and distances were kept in millimetres
   internally, converting only when an API needs another unit.
3. An IO connection table was prepared. The user asked about isolated sensor,
   motor, and encoder checks and about connecting to the robot using its
   network.
4. Empty Shuffleboard/SmartDashboard views were investigated. A compact
   purpose-built Shuffleboard view replaced reliance on automatic LiveWindow
   widgets; numeric sonar range entries were separated from Boolean digital
   pin states. A detailed telemetry option remained available for diagnosis.
5. The sonar polling path was checked after invalid readings. It uses four
   distinct trigger/echo pairs and one common polling implementation. The
   VMX IO watchdog's disabled behavior became a key diagnostic finding; a
   powered sensor can still lack trigger pulses while Disabled.
6. An encoder-only 1000 mm test was added, with start-button debounce,
   per-run encoder baselines, ramping, near-target slowdown, stall/reverse/
   disagreement/timeout checks, and a minimal Forward 1m dashboard.
7. The first arena attempt went left about 535 mm and was incomplete at
   525.97 mm encoder travel. The distance scale was left unchanged; an
   empirical -90° drive correction, bounded two-wheel synchronization, and
   persistent post-stop status were then added. No confirming repeat run has
   been reported yet.
8. Manual teleoperation is now the source default. It reads the Control
   Station's first joystick, uses the left stick for robot-relative translation
   including diagonals, and reports raw axes/encoder/output values on a new
   Teleop tab. Debug and release builds succeeded after this source change.

## 11. Current files and responsibilities

| File | Responsibility |
|---|---|
| `src/main/include/LocalizationConstants.h` | Mode selection, IO mappings, wheel/sensor geometry, joystick axis/sign mapping, output/speed limits, arena placeholders, sensor-fusion tuning. |
| `src/main/include/Hardware.h`, `src/main/cpp/Hardware.cpp` | VMX DIO inputs, ultrasonic objects, navX access, Titan M0-M2 and encoder access, wheel-kinematic output and motor stop behavior. |
| `src/main/include/DriveKinematics.h` | Shared wheel speed and direction-projection equations. |
| `src/main/include/ForwardDriveController.h` | Hardware-independent one-metre state machine and encoder feedback for the optional exercise. |
| `src/main/include/Localization.h`, `src/main/cpp/Localization.cpp` | Pose, encoder odometry, yaw/acceleration fusion, predicted sonar wall ranges, covariance correction, optional detailed telemetry. |
| `src/main/include/subsystems/LocalizationSubsystem.h`, `src/main/cpp/subsystems/LocalizationSubsystem.cpp` | Periodic mode selection, controller input processing, subsystem/dashboard setup, sonar ping scheduler, localization start and diagnostics. |
| `src/main/cpp/Robot.cpp` | Robot lifecycle, cleanup of old SmartDashboard trees, LiveWindow suppression, automatic tab selection. |
| `src/main/cpp/RobotContainer.cpp` | Command/subsystem container; no autonomous command is currently selected. |
| `SkillsChallenge-Info.md` | Operator instructions and source-level wiring/measurement documentation. |
| `src/test/cpp/ForwardDriveControllerChecks.cpp` | Compile-time regression checks for the pure one-metre controller. These are not a substitute for a physical drive test. |

## 12. Suggested first teleop check

1. Keep the robot disabled, place it on a stand or otherwise ensure the wheels
   can move without the chassis driving into an obstacle, and connect the
   remote to the Windows PC.
2. Open Control Station and verify its Joysticks indicator. Open the Teleop
   Shuffleboard tab and confirm the controller and raw left X/Y values.
3. With the robot still disabled, move the stick in each direction and record
   the raw axis signs. If needed, edit the joystick port, axis indices, or
   signs in `LocalizationConstants.h`, then rebuild and redeploy.
4. Enable Teleoperated with the stick centered. Confirm **Stick armed**.
   Briefly test each direction at low risk, checking motor outputs and encoder
   count changes. Disable immediately if the chassis direction differs from
   the intended command; correct the configured frame or motor signs before
   extending the test.
5. Confirm the -90° physical frame correction with a short forward movement
   before relying on it. Record whether the robot translates straight and
   whether it rotates. Only after direction, motor polarity, and frame
   alignment are understood should the one-metre or sensor-fusion mode be
   treated as calibrated.

This checklist is a bring-up aid, not a record that these physical checks have
already passed. The last confirmed source build does not prove controller axis
mapping, motor polarity, encoder calibration, sonar timing, or arena pose
accuracy on the installed robot.
