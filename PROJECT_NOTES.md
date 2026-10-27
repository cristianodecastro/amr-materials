# Project 7: robot software, decisions, and operating notes

**23 October 2026 — Cobra check:** the user connected the Cobra through its
analog module J1 to VMX I2C. Enabled the existing Cobra driver and added a
dedicated Shuffleboard check with raw voltages, disabled-only averaged white
and black captures, per-channel contrast checks, and independent BLACK lamps.
Captures are temporary diagnostic references, not autonomous calibration.
Autonomous tape validity is now explicitly gated by COBRA_CALIBRATED, which
remains false. Debug/release assemble passed; no automatic deployment or
physical sensor result is claimed. Procedure: [COBRA_CHECK.md](COBRA_CHECK.md).

This document records what was designed and implemented during the project
work in this chat, why the design changed over time, how the VMX, Titan,
sensors, controller, and dashboards are connected, what has actually been
observed on the robot, and what remains to be calibrated. It describes the
development of `7 - SkillsChallenge/SkillsChallenge`.

**Autonomous update:** the current arena design and operating instructions are
in [AUTONOMOUS_SETUP.md](AUTONOMOUS_SETUP.md). The former one metre controller
has been deleted. Earlier observations below are retained as history; old
calibration values and former dashboard names are superseded by that guide.

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

The software now selects operation from Control Station mode:

1. **Teleoperated:** left-stick translation and L2 CW / R2 CCW rotation.
2. **Autonomous:** mapped waypoint navigation, tape docking, action hooks and
   job sequencing. Unconfirmed geometry/calibration and the unfinished pickup
   mechanism currently block start.
3. **Stationary observation:** disable manual drive in source and enable
   Teleoperated mode to inspect sensors while commanding drive outputs zero.

Localization remains observable in manual operation; wall corrections are
restricted to initialized autonomous operation. The old one metre and
localization-only mode selectors were removed.

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

L2 and R2 use digital button inputs, numbered from 1 by WPILib. Defaults are
`DRIVER_L2_BUTTON = 7` and `DRIVER_R2_BUTTON = 8`, a PS-style mapping. The exact
Windows/Control Station mapping is not established by the product page.
Check the L2/R2 dashboard lamps while Disabled, and change these two constants
if the physical buttons report different numbers in Windows `joy.cpl`.
Trigger-axis values such as -1 at rest are not used by this digital mapping.

### Motor specifications, translation shaping, and rotation

- A circular 12% center deadband suppresses stick noise. Values outside the
  deadband are rescaled smoothly to the requested speed range.
- The vector magnitude is capped, so a full diagonal does not request
  sqrt(2) times the maximum straight-line speed.
- The full-stick request is about 598.16 mm/s, calculated from a 102 mm wheel
  diameter and 112 RPM maximum no-load gearbox-output speed. A 1000 mm/s^2
  limit smooths translation changes. Centering the stick requests zero
  translation immediately, including while rotation is active.
- Manual drive currently allows 90% motor output. `Hardware::DriveRobotRelative()`
  maps translation and yaw through the three-wheel inverse kinematics, and
  `DriveWheelsMMps()` scales the combined wheel vector together if a motor
  reaches its limit. This preserves the requested ratios while reducing
  translation and yaw together during saturation.
- L2 requests negative (clockwise) yaw; R2 requests positive (counterclockwise)
  yaw. The nominal yaw limit is 90 degrees/s and its ramp is 180 degrees/s^2.
  Both triggers pressed cancel yaw. Releasing both triggers commands zero yaw
  immediately while translation can continue. The left stick and triggers
  can be used simultaneously. `TELEOP_YAW_SIGN = +1` follows the existing
  tangent-wheel convention; the installed rotation direction still needs a
  physical check.
- The mode does not implement heading hold, field-oriented driving, obstacle
  avoidance, sonar correction, or a measured velocity PID loop. The RPM-based
  wheel-speed scale is a feedforward estimate, so dashboard speeds remain
  requests rather than verified physical speeds under load.
- The physical Start button is unused in this mode. Wheel encoder counts are
  displayed for observation, but they do not adjust the teleop command.

After Control Station attaches, the controller must report both configured
axes and enough buttons for the configured L2/R2 numbers. A missing Driver
Station or missing configured inputs commands zero and disarms the
stick. Disabled, Autonomous, or Test mode also commands zero and disarms it.
After enabling, changing modes, or reconnecting, center the stick until
**Stick armed** is true and release both L2/R2 before moving. Arming requires
the stick centered and both triggers released, so holding a control through
enable cannot immediately start the robot. If a PC continues to send
stale axis values as valid controller input after a Bluetooth failure, the
robot cannot independently detect the physical radio failure; the status
reflects the data reported through Control Station.

Studica's [Maverick motor specifications](https://docs.wsr.studica.com/en/latest/docs/Motors/maverick-dc-motor.html)
give no-load speed 88/100/112 RPM (minimum/nominal/maximum) and rated speed
68/80/92 RPM. These are already speeds at the 61:1 gearbox output. For direct
drive, wheel surface speed is `pi * 102 * RPM / 60` mm/s. Nominal no-load speed
is about 534.07 mm/s and nominal rated speed about 427.26 mm/s. The code uses
the maximum no-load value, 598.16 mm/s, as the full-output wheel scale; it does
not divide the output-shaft RPM by 61 again. External gearing still belongs
in `MOTOR_TO_WHEEL_RATIO`.

The first manual-drive revision limited output to 25% and used a 1000 mm/s
placeholder. With a 250 mm/s request, full forward applied only about 21.65%
to M0/M1; full sideways translation reached 25%. The updated full forward
pattern reaches about +86.6%/-86.6%/0 and sideways travel can reach the current 90% cap on
the fastest wheel. These are ideal command percentages, not measured chassis
speeds. The earlier one-metre exercise used its own 25% cap; that controller
has now been removed. Autonomous has a separate calibrated-speed control path.

### Teleop dashboard

Robot startup selects the **Teleop** Shuffleboard tab. Its 18 values are
arranged in four rows:

1. Drive status, Enabled, Controller, Stick armed.
2. Raw left-stick X/Y and requested forward/left speeds in mm/s.
3. M0/M1/M2 raw encoder counts and applied output percentages.
4. L2 - CW, R2 - CCW, requested yaw rate in degrees/s, and output limit (%).

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

## 5. Retired one-metre exercise

The button-started encoder-only exercise was useful for exposing a direction
mismatch. Its controller, constants, dashboard and dedicated checks have been
deleted as part of the arena autonomous work. The observed encoder counts and
direction correction are preserved in section 7 as development evidence.

## 6. Current localization and autonomous integration

See [AUTONOMOUS_SETUP.md](AUTONOMOUS_SETUP.md) for current parameters,
coordinate conventions, route planning, wall/tape corrections, hardware hooks,
start checks and fault behavior. Highlights:

- Body-to-wheel commands and inverse encoder odometry now share the -90 degree
  empirical frame correction. This still needs physical confirmation.
- Encoders drive translation; gyro/AHRS drives yaw. Acceleration is captured
  but its integration weight is now zero to prevent bias drift.
- Wall raycasting includes both central divider segments. Object envelopes
  occlude walls and are not trusted localization landmarks.
- Sonars are staggered, require Control Station enable, and only correct pose
  during suitably slow motion with fresh, geometrically consistent readings.
- Cobra uses per-channel floor/tape calibration and station-specific docking.
- The Autonomous and Calibration tabs replace the old Robot/Forward layouts.

Earlier sonar troubleshooting found that Boolean LiveWindow DIO widgets did
not represent distance, and VMX's disabled IO watchdog could suppress triggers
despite 5 V sensor power. Numeric mm ranges and explicit enabled-state polling
resolved that display/trigger confusion. The left sonar later appeared
unavailable on its own; it uses the same production path as the others on
DIO 4/5, and no confirmed hardware root cause was established in this chat.
These observations remain relevant to calibration; ping requests alone do not
prove a signal was measured electrically.

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
| Titan output counts/revolution | 1464 | Matches Studica's documented 6 CPR * 61 * 4 quadrature output-shaft convention; verify readings on the installed hardware. |
| Motor-to-wheel ratio | 1.0 | Assumes direct drive; verify external gearing. |
| Encoder and motor signs | all +1.0 | Must agree with installed motor and encoder wiring; the first drive observation makes this worth rechecking. |
| Full-output wheel surface-speed scale | 598.16 mm/s | 112 RPM at 102 mm diameter, external ratio 1.0; no-load feedforward estimate. |
| Teleop full-stick request / output cap | 598.16 mm/s / 90% | Replaces the initial 250 mm/s request and 25% cap; reflects the user's current 90% setting. |
| L2/R2 button mapping | 7 / 8 | PS-style digital defaults; verify with the Disabled dashboard lamps. |
| Teleop yaw rate / acceleration | 90 degrees/s / 180 degrees/s^2 | Nominal requests, L2 CW and R2 CCW; both triggers cancel. |
| Arena size | 2000 x 2000 mm | Placeholder. |
| Starting pose | (500, 250) mm, 0° | Placeholder. |
| Sonar mount faces | rear X=-100 mm, rear half-spacing 80 mm, side Y=120 mm | Symmetric placeholders; measure actual sensing faces from robot center. |
| Sonar beam directions | rear 180°, left +90°, right -90° | Geometric assumptions; verify actual sensor orientation and names. |
| Joystick mapping | port 0, axes 0/1, signs +1/-1 | Common default; verify in Windows `joy.cpl` and the Teleop raw-axis cards for the selected USB/Bluetooth mode. |
| Manual drive frame correction | -90° relative to source wheel-layout frame | Empirical, based on the first one-metre attempt moving left; not yet confirmed by a subsequent physical run. |

The one-metre estimate uses the documented output-count scale
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
9. After the user reported slow manual translation, the wheel-speed scale was
   corrected to the Maverick's documented 112 RPM maximum no-load output
   speed, and the manual motor cap was raised from 25% to 100%. L2/R2 rotation,
   combined translation/rotation, trigger-aware arming, four more dashboard
   fields, and a pure teleop controller with compile-time checks were added.

## 11. Current files and responsibilities

| File/class | Responsibility |
|---|---|
| LocalizationConstants | Wheel/IMU/body frames, IO, manual controls and sonar configuration |
| AutonomousConfig | Arena, stations, jobs, footprint, Cobra, tuning, confirmation flags |
| ManipulatorConfig | Future Maverick/servo/limit placeholders |
| RobotGeometry / DriveKinematics | Shared pose and frame/wheel calculations |
| ArenaMap | Inflated-obstacle planning and trusted/untrusted sonar ray hits |
| AutonomousController | Nonblocking mission states, routes, tape docking and action requests |
| Hardware | VMX/Titan/Cobra access, manual mixer, autonomous wheel-speed PI and object hooks |
| Localization | Encoder/AHRS pose, covariance, gated wall/tape updates |
| LocalizationSubsystem | Mode ownership, polling, controller input/output and dashboard tabs |
| Robot / RobotContainer | Scheduler and explicit autonomous/disable/mode-entry lifecycle |
| TeleopDriveController | Existing input shaping, neutral arming and trigger yaw |
| TeleopDriveControllerChecks.cpp | Existing compile-time manual-controller/mixer checks |
| AUTONOMOUS_SETUP.md | Current autonomous architecture, status, calibration and operations |
| SkillsChallenge-Info.md / TELEOP_SETUP.md | Current overview/wiring and manual operation |

The retired ForwardDriveController and template Example command/subsystem
files are no longer present. Project 7 remains independent of projects 1–6.

## 12. Suggested first teleop check

1. Keep the robot disabled, place it on a stand or otherwise ensure the wheels
   can move without the chassis driving into an obstacle, and connect the
   remote to the Windows PC.
2. Open Control Station and verify its Joysticks indicator. Open the Teleop
   Shuffleboard tab and confirm the controller and raw left X/Y values.
3. With the robot still disabled, move the stick in each direction and record
   the raw axis signs. If needed, edit the joystick port, axis indices, or
   signs in `LocalizationConstants.h`, then rebuild and redeploy.
4. While Disabled, verify the L2/R2 lamps match their physical triggers. Enable
   Teleoperated with the stick centered and both triggers released. Confirm **Stick armed**.
   Briefly test each direction at low risk, checking motor outputs and encoder
   count changes. Disable immediately if the chassis direction differs from
   the intended command; correct the configured frame or motor signs before
   extending the test.
5. Confirm the -90° physical frame correction with a short forward movement
   before relying on it. Record whether the robot translates straight and
   whether it rotates. Only after direction, motor polarity, and frame
   alignment are understood should autonomous navigation be considered for
   calibration. Follow the current autonomous guide.

This checklist is a bring-up aid, not a record that these physical checks have
already passed. The last confirmed source build does not prove controller axis
mapping, motor polarity, encoder calibration, sonar timing, or arena pose
accuracy on the installed robot.

## 13. Control Station connection recovery — 20 October 2026

The user reported that Windows could join the robot Wi-Fi but Control Station
could no longer connect after the previous teleoperation session. Diagnosis
was performed on the PC and on the VMX, and communication was restored.

### Findings

- `VMX-23` was broadcasting. Connecting the saved Windows profile assigned the
  PC `10.0.23.3/24`. The VMX at `10.0.23.2` answered ping and SSH, proving that
  the access point, IP addressing, and basic OS networking were working during
  those checks. One association attempt failed transiently, but subsequent
  association and SSH succeeded.
- NetworkTables TCP port 1735 was not reachable, and the robot was not listening
  for Driver Station packets on UDP 1110. Control Station had enabled Public
  firewall allow rules; a missing app exception was not established as the
  cause.
- On the robot, `vmx_robot_manager.service` was enabled for boot but inactive.
  `/home/lvuser/robotCommand` referenced `/home/lvuser/frcUserProgram`, which was
  missing. The launch log explicitly reported `No such file or directory`.
- Direct Ethernet then provided a stable diagnostic path: PC address
  `172.22.11.1/24`, robot address `172.22.11.2`.
- A forced deployment restored the executable and native libraries. The first
  startup then failed to find `libopencv_videoio.so.3.4`. That library was
  present, had the correct ARM architecture, and matched the local cached file
  by SHA-256. Refreshing the dynamic-loader cache with an explicit privileged
  `/sbin/ldconfig` made all executable dependencies resolve. The existing
  `/etc/ld.so.conf.d/frc_libs.conf` already contained the correct library path;
  no new loader configuration was needed.

### Recovery performed

The stale Control Station session was closed so no driver enable command would
be present during recovery. From this project's directory, the existing
release program was redeployed over Ethernet:

```powershell
$env:JAVA_HOME = 'C:/Users/Public/wpilib/2020/jdk'
.\gradlew.bat deploy -PteamNumber=23 --offline -Pdeploy-dirty --console=plain
```

The `deploy-dirty` property forced files to be transferred rather than relying
on the deployment cache. Gradle discovered `pi@172.22.11.2:22` automatically;
the golden project configuration was not edited. After deployment, the
following recovery commands were run on the VMX through SSH:

```sh
sudo /sbin/ldconfig
sudo systemctl restart vmx_robot_manager.service
sync
```

Verification showed an active robot-manager service, startup enabled at boot,
a running `/home/lvuser/frcUserProgram`, no unresolved `ldd` dependencies,
UDP 1110 bound by the robot program, and TCP 1735 listening and reachable
from the PC. The deployed executable's SHA-256 matched the local release
artifact. Control Station was reopened; the user entered `172.22.11.2` and
confirmed that **Robot Comms and Robot Code were both checked**, with the robot
kept Disabled during the check.

For subsequent connections, use `172.22.11.2` with the Ethernet cable, or join
`VMX-23` and use `10.0.23.2` over Wi-Fi. These are different interfaces of the
same robot. The temporary Wi-Fi diagnostic connections were restored to the
PC's original network; Ethernet allowed robot access while preserving the
PC's Internet connection.

### If the symptom returns

First distinguish a Wi-Fi association/IP problem from a robot-program startup
problem. With the PC on the appropriate network, ping the matching robot IP,
then use SSH to check:

```sh
ls -l /home/lvuser/frcUserProgram
systemctl status vmx_robot_manager.service --no-pager
tail -n 80 /var/local/kauailabs/log/FRC_UserProgram.log
ldd /home/lvuser/frcUserProgram
```

An absent executable requires a successful redeployment. A present executable
with unresolved libraries requires checking deployment of its dependencies
and refreshing the loader cache before restarting the manager. A successful
Gradle result alone did not establish successful runtime startup in this
incident; the process, sockets, log, and Control Station indicators were
checked afterward.

The available evidence does not establish why the executable had disappeared.
The native deployment sequence deletes the previous executable before
uploading its replacement, so an interrupted deployment is one possible
explanation, not a confirmed cause. Motor direction, physical speed, and
controller mapping were not tested as part of this connection recovery.

## 14. Trigger direction swap — 20 October 2026

The user requested that the two rotation controls exchange their purposes.
The current mapping is **L2 = clockwise** and **R2 = counterclockwise**.
The production controller now computes `(r2 ? 1 : 0) - (l2 ? 1 : 0)` for its
signed rotation request. Its status message, the Shuffleboard L2/R2 direction
labels, the teleop regression checks, this document, `TELEOP_SETUP.md`, and
`SkillsChallenge-Info.md` were updated to the same mapping.

`TriggerDirections()` is a regression check, not the input-binding code.
`Settled(x, y, l2, r2)` presses simulated controls to test the real production
controller. Changing only the Boolean arguments in that test would not alter
what the robot does. Related tests were updated to check CW/CCW signs, mixed
translation/rotation, yaw acceleration, and wheel kinematics with the new
mapping. The test that previously required an exact 100% cap was removed,
and output-limit checks use the configured cap; the user's existing 90%
source setting was preserved.

## 15. Arena autonomous foundation — 20 October 2026

The user supplied a 2000 mm arena drawing with a central start square, split
divider, four upper storage stations and four lower deliveries. The old
distance exercise was removed and replaced with a fixed-map mission state
machine. Routing inflates obstacles by a circular robot/cargo envelope and
clearance; the central square remains a traversable floor marking.

The controller navigates to station staging points, acquires expected Cobra
tape, docks with odometry and line feedback, waits for confirmed manipulation,
retreats, and proceeds to the assigned delivery. It stops for route/timing,
sensor, uncertainty, tape, progress, proximity or mechanism faults. A new
Autonomous entry requires repositioning and a fresh physical Start sequence.

The user confirmed one additional Maverick motor and two servos, with possibly
one or two limit switches. Detailed mechanism behavior/ports remain unknown,
so only explicit hooks/placeholders were added. The user will provide a
specific storage assignment/order and cannot yet confirm start heading,
footprint or Cobra mounting/version. These remain blocking configuration
requirements. No complete pickup/release is claimed.

Autonomous gets feedforward + PI wheel-speed control; manual drive retains its
90% cap, controls and open-loop behavior. Existing project/vendor/Gradle
parameters were preserved. Debug and release assemble succeeded. No physical
arena run or deployment of this new autonomous program was performed.

