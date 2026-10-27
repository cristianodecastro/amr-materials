# Teleoperation: speed and L2/R2 rotation

## Current controls

| Input | Request |
|---|---|
| Left stick up/down | Forward/backward translation |
| Left stick left/right | Left/right translation |
| Left stick between axes | Diagonal translation, with magnitude limited to the straight-line request |
| L2 held | Clockwise rotation |
| R2 held | Counterclockwise rotation |
| L2 and R2 together | Zero rotation; stick translation can continue |
| Center stick | Zero translation; held-trigger rotation can continue |
| Release triggers | Zero rotation; stick translation can continue |

After enabling or reconnecting, center the stick and release both triggers to
arm. The dashboard's existing **Stick armed** field now covers all three
controls. Driver Station/controller loss or leaving enabled Teleoperated mode
commands zero and disarms. Releasing a control commands its component to zero
immediately; the chassis can still coast mechanically.

## Why translation was slow and what changed

The first manual-drive version requested 250 mm/s against an uncalibrated
1000 mm/s wheel scale, with a separate 25% motor cap. Full forward consequently
applied only about 21.65% on M0/M1; sideways travel topped out at 25% on M2.

The [official Maverick 75001 specifications](https://docs.wsr.studica.com/en/latest/docs/Motors/maverick-dc-motor.html)
state 88/100/112 RPM no-load and 68/80/92 RPM rated-load speeds. Those values
already include the 61:1 gearbox. With the user's 102 mm wheels and external
motor-to-wheel ratio 1.0:

```text
wheel surface speed (mm/s) = pi * 102 * output RPM / 60
maximum no-load (112 RPM)  = 598.16 mm/s
nominal no-load (100 RPM)  = 534.07 mm/s
nominal rated (80 RPM)     = 427.26 mm/s
```

`MAX_WHEEL_SPEED_MMPS` is now calculated from wheel radius, 112 RPM, and the
external ratio. `TELEOP_SPEED_MMPS` uses that same scale and the teleop output
cap is currently 90%. The left-stick vector ramp is 1000 mm/s^2. Full forward nominally
requests M0 +86.6%, M1 -86.6%, M2 zero; other headings have different wheel
ratios and can use up to the configured 90% cap. These are feedforward requests, so actual speed under
load depends on the motor, battery, surface, and chassis. No encoder velocity
PID or voltage compensation was added. The maximum no-load speed is not a
guarantee of chassis speed.

The -90-degree mounting correction from the previous arena exercise remains
the manual translation frame. It is now shared by hardware command conversion
and inverse encoder odometry through `DRIVE_FORWARD_DIRECTION_DEG`.
Its physical alignment still needs confirmation.
Rotation follows the tangent-wheel model's CCW-positive convention and uses
the user's 168 mm wheel-center radius. `TELEOP_YAW_SIGN` can be reversed if a
physical rotation check establishes opposite installed polarity.

## L2/R2 mapping and rotation speed

The source reads digital `GetRawButton()` inputs, using **L2=7 and R2=8** as
PS-style defaults. Button numbers are one-based. These numbers have not been
confirmed on the user's Windows USB/Bluetooth controller mode. While Disabled,
press L2 and R2 separately and check the **L2 - CW** / **R2 - CCW** lamps in
Shuffleboard. Change `DRIVER_L2_BUTTON` / `DRIVER_R2_BUTTON` in
`src/main/include/LocalizationConstants.h` if the numbers differ in Windows
`joy.cpl`. The controller must expose both configured axes and the configured
button numbers; otherwise the program reports missing controller inputs and
disarms. This digital implementation does not interpret trigger-axis ranges
or provide analog trigger pressure control.

The default yaw request is **90 degrees/s** with a **180 degrees/s^2** ramp.
L2 requests -yaw (CW), R2 requests +yaw (CCW). Both triggers cancel yaw immediately.
Translation and yaw can be requested simultaneously. When the combined wheel
requests exceed the output cap, all wheel commands are scaled by the same
factor, reducing both translation and rotation while preserving their ratios.
There is no automatic heading hold.

The mapping was reversed on 20 October 2026 at the user's request. The
production mapping is the `rotation` expression in
`src/main/include/TeleopDriveController.h`:

```cpp
const int rotation = (r2 ? 1 : 0) - (l2 ? 1 : 0);
```

`Settled(x, y, l2, r2)` in `TeleopDriveControllerChecks.cpp` supplies simulated
inputs to the real controller. Changing those test arguments alone would
change what the test presses; it would not change the robot's controls.
The regression checks, controller status, dashboard direction labels, and
project documentation now use L2=CW and R2=CCW.

## Deploy and observe

From this project's folder:

```powershell
$env:JAVA_HOME = 'C:/Users/Public/wpilib/2020/jdk'
.\gradlew.bat deploy -PteamNumber=23 --offline
```

Connect the remote to the Windows PC running Control Station, connect Control
Station to `10.0.23.2`, select Teleoperated (`o`), and use `e`/`d` to enable or
disable. No physical Start press is needed. In the **Teleop** tab, the fourth
row now contains the two trigger lamps, yaw request in degrees/s, and output
limit. The first three rows keep the existing status, raw axes, forward/left
requests, encoder counts, and applied output percentages.

## Source and verification

`TeleopDriveController.h` owns input shaping, validation, arming, and yaw
requests without depending on VMX or dashboard APIs. The subsystem reads the
Driver Station inputs and applies that controller's results to the hardware.
`DriveKinematics.h` contains the shared wheel equations and the output scaling
used by `Hardware::DriveWheelsMMps()`.

Compile-time checks evaluate the production controller and motor mixer:

```powershell
& 'C:/Users/Public/wpilib/2020/roborio/bin/arm-frc2020-linux-gnueabi-g++.exe' -std=c++14 -Wall -Wextra -I src/main/include -fsyntax-only src/test/cpp/TeleopDriveControllerChecks.cpp
```

Checks cover motor RPM conversion, neutral arming, held triggers, L2/R2 yaw
signs, simultaneous trigger cancellation, cardinal/diagonal translation,
combined motion, independent release, disable/disconnect, invalid input, ramp
limits, frame separation, and saturated wheel output. The former forward-run
controller and its check file have been deleted. Earlier checks passed before
the autonomous work; debug/release `assemble` also succeeded with the new code.
The controller button mapping, actual chassis speed, and installed rotation
direction still need confirmation on the robot.

## Autonomous integration

Control Station now selects manual versus autonomous operation at runtime.
Teleop retains its 90% cap and L2 CW / R2 CCW mapping. Encoders, IMU and sonars
remain observable in manual mode; arena wall corrections and autonomous motion
do not run there. See [AUTONOMOUS_SETUP.md](AUTONOMOUS_SETUP.md) for the arena
map, calibration gates, pickup integration and new dashboard tabs.
