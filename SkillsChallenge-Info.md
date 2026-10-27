# VMX three-wheel arena project

Project 7 is an independent WPILib 2020 C++ project. Projects 1–6 were references;
none is imported, linked or required at runtime. The generated Gradle/vendor
configuration remains unchanged by the autonomous work. All linear values use
millimetres, millimetres/second or millimetres/second²; time is seconds, angles
are degrees at configuration/IO and radians in calculations.

## Current software

- **Teleoperated:** robot-relative left-stick translation, including diagonals;
  L2 clockwise and R2 counterclockwise. Existing 90% cap is preserved.
- **Autonomous:** arena navigation, tape docking, pickup/delivery action hooks,
  retreat and job sequencing. Calibration and mechanism readiness currently
  block start. The complete retrieval mission is not yet physically usable.
- Encoders, IMU and sequential sonars remain observable. Wall corrections are
  used in autonomous operation after arena initialization.
- The former one metre controller and unused template commands were deleted.

Control Station selects mode at runtime. `TELEOP_DRIVE_ENABLED` enables manual
driving only; it no longer takes priority over Autonomous mode.

## Guides

- [Autonomous arena model, algorithm and configuration](AUTONOMOUS_SETUP.md)
- [Teleop connection, controls and speed](TELEOP_SETUP.md)
- [Cobra black-tape check and white-floor calibration](COBRA_CHECK.md)
- [Development history and connection recovery](PROJECT_NOTES.md)
- [Provisional arena diagram](docs/arena-model.svg)

Use **Teleop**, **Autonomous**, **Calibration** and **Cobra check** tabs in Shuffleboard.
Old saved Robot/Forward/LiveWindow widgets do not describe the current layout.
The Calibration tab separates raw Cobra voltages from the operating display.
The Cobra check tab is the current startup view and provides white/black
captures and four independent BLACK indicators while Control Station is Disabled.
SmartDashboard detail is optional, disabled by default.

## Current wiring

These are WPI logical channel assignments. Match them to the actual VMX pins
using the [Studica WPI addressing diagrams](https://docs.wsr.studica.com/en/latest/docs/VMX/wpi-channel-addressing.html);
WPI numbers and physical pin numbers differ, and jumper configuration matters.
Existing FlexDIO sonar wiring is preserved.

| Device | Assignment | Connection |
|---|---|---|
| Rear-left sonar | Trigger DIO 0 / echo DIO 1 | Existing FlexDIO pair, A trigger / B echo |
| Rear-right sonar | Trigger DIO 2 / echo DIO 3 | Existing FlexDIO pair, A trigger / B echo |
| Left sonar | Trigger DIO 4 / echo DIO 5 | Existing FlexDIO pair, A trigger / B echo |
| Right sonar | Trigger DIO 6 / echo DIO 7 | Existing FlexDIO pair, A trigger / B echo |
| Start | DIO 8, active low | Normally-open switch from input to ground |
| Quadrant II wheel, radial 150 deg | Titan M0 / encoder 0 | Motor and matching Titan encoder connector |
| Quadrant I wheel, radial 30 deg | Titan M1 / encoder 1 | Motor and matching Titan encoder connector |
| Bottom wheel, radial 270 deg | Titan M2 / encoder 2 | Motor and matching Titan encoder connector |
| Titan communications | CAN ID 42 | VMX CANH/CANL to matching Titan signals |
| IMU | VMX navX, existing SPI kMXP API | Onboard; no new wiring |
| Cobra with analog module | Four channels through Cobra analog module | User connected module J1 to VMX I2C; reads enabled, calibration/mount geometry pending |
| Mechanism Maverick motor | Proposed spare Titan M3 | Not yet confirmed/instantiated |
| Two servos | PWM channels -1/-1 | Unassigned placeholders; no outputs instantiated |
| Possible limit switches | DIO channels -1/-1 | Unassigned placeholders; no inputs instantiated |

Use the specified sonar supply and common ground; the existing sensors use
5 V. Retain the established VMX jumper settings. The
[Studica Cobra guide](https://docs.wsr.studica.com/en/latest/docs/Sensors/cobra.html)
describes its analog-module/I2C connection. The connected analog-module path
is enabled for diagnostic readings; follow COBRA_CHECK.md before using tape
classification in autonomous operation.

All wheel encoders are read over Titan CAN, not through the sonar DIO pairs.
The future motor, servo and limit channels must be assigned together to avoid
physical pin conflicts. In particular, WPI PWM and DIO channel indices are
different namespaces that may refer to the same physical pin.

## Sonar observation without driving

Sonar trigger pulses require Control Station enable because of the VMX IO
watchdog. A numeric -1 means disabled, invalid or no echo. Generic LiveWindow
DigitalInput booleans are not sonar distances.

For a stationary observation session, set `TELEOP_DRIVE_ENABLED=false`,
build/deploy, select Teleoperated mode and enable. The subsystem commands
drive outputs zero and still polls sonars. Use the Autonomous tab's four range
widgets. Autonomous mode has separate start checks; disabling manual drive
does not disable a fully configured autonomous mission. Restore manual driving
after the observation session.

One ping at a time, at least 60 ms echo settling and 40 ms quiet spacing, avoids
competing automatic sonar threads. An entire cycle takes roughly 400 ms.
Physical Start is not required for raw readings.

## Build and connections

Open this folder, containing `build.gradle` and `gradlew.bat`, as the project.
Use installed WPILib 2020 Java/toolchain and matching VMX runtime:

```powershell
$env:JAVA_HOME = 'C:/Users/Public/wpilib/2020/jdk'
.\gradlew.bat assemble -PteamNumber=23 --offline --console=plain
```

This compiles and links; it does not deploy. Current connection recovery used
direct Ethernet: PC 172.22.11.1/24 and robot 172.22.11.2. Robot WiFi's configured
address is 10.0.23.2. Control Station takes the address of the interface in use.
The detailed recovery notes explain the prior missing robot executable and
runtime library-cache repair.

See the autonomous guide before deployment or an arena run. Geometry and
mechanism placeholders intentionally prevent autonomous motion.
