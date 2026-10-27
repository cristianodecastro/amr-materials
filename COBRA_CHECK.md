# Cobra check: black tape on the white wooden floor

## What changed

The connected analog-module interface is enabled with `COBRA_INSTALLED=true`.
Hardware reads the four channels through the existing `studica::Cobra` driver.
The new **Cobra check** Shuffleboard tab shows raw voltages and a separate,
session-calibrated black/white classifier. It is selected when the program boots.

This check does not move the robot. Keep Control Station **Disabled** for the
whole procedure. Normal teleop remains available if you deliberately enable
Teleoperated mode; no drive settings were changed for this check.

The autonomous `COBRA_CALIBRATED` flag remains false. Its tape tracker no longer
reports a valid line from placeholder floor/tape values. The diagnostic lamps
are independent of that narrow-line tracker: all four BLACK lamps may be green
on a wide tape patch even though such a patch is not a valid narrow line.

## Physical setup

- The Cobra connects to the analog module, then the module's I2C connection
  goes to the VMX connector marked I2C, using the correct supplied cable.
  Verify connector functions rather than relying only on a J1/J2 identifier.
- Use the configured 5 V supply/reference. Do not power Cobra from the servo
  block's 6 V outputs.
- For the older Cobra, start with its sensing faces approximately **3 mm**
  above the actual floor/tape surface, parallel to it. This is measured from
  the optical sensor faces, not the PCB top or mounting bracket.
- Keep the mounting height and arena lighting unchanged during calibration
  and evaluation. Wooden-floor grain, glossy tape and shadows can affect IR
  reflection; visible black does not guarantee strong IR contrast.

Studica documents the analog module to VMX I2C path, four voltage channels,
and typical 3 mm sensing height in its
[Cobra guide](https://docs.wsr.studica.com/en/latest/docs/Sensors/cobra.html).

## Build/deploy once

From the `SkillsChallenge` folder, with Control Station Disabled:

```powershell
$env:JAVA_HOME = 'C:/Users/Public/wpilib/2020/jdk'
.\gradlew.bat deploy -PteamNumber=23 --offline --console=plain
```

Use the existing working Ethernet/WiFi connection and deployment setup. This
change was built locally but was not automatically deployed to the robot.

## Capture and evaluate

1. Open Shuffleboard, connect to the robot's active IP and choose **Cobra check**.
   Keep Control Station Disabled. No physical Start press is needed.
2. Place **all four sensing elements over bare white wooden floor** at normal
   installed height. Verify that `CH0..CH3 live (V)` update. Click
   **Capture white** and hold still for about one second.
3. Place **all four elements over black tape of the same type used in the
   arena**. Make a wide patch using adjacent strips if a single strip does
   not cover the entire array. Keep the same sensing height. Click
   **Capture black**, then hold still for about one second.
4. Wait for **Ready: sweep tape across CH0..3; green means black**.
   The program requires at least 0.30 V difference between captured white and
   black on every channel, and rejects captures varying by over 0.20 V while
   held stationary. These are initial diagnostic thresholds, not sensor specs.
5. Return to white: all four BLACK lamps should be off/red. Move a tape edge
   slowly under each element in turn: its lamp should turn on/green, then off
   again over white. A wide black patch should turn all four on.
6. Repeat under normal arena lighting and at several floor positions. Check
   repeatability and record which channel is physically leftmost/rightmost.

The button automatically returns to off after each request. The program
averages at least 10 samples over at least 0.75 s per capture. Calibration is
accepted only while Disabled. Enabling during a capture cancels it.

Black may produce a higher OR lower voltage than white. The classifier learns
the polarity separately for each element, using the midpoint between its white
and black references. There is no assumed universal voltage threshold.

## Interpreting problems

| Observation | Meaning / next step |
|---|---|
| Live values are -1 | Interface is absent in the running code or reading is unavailable; confirm the updated binary is deployed |
| Values outside the expected 0…5 V range | Check supply/reference, cable and connections; power off before changing wiring |
| All values stay at zero or another constant on both surfaces | No demonstrated surface response; check Cobra-to-module connection, module-to-I2C cable, power and sensor height |
| Only one element fails to change | Inspect that sensor's optical face, height and corresponding analog connection |
| Low white/black contrast | Recheck height, actual floor/tape materials and whether the full array was over each calibration surface |
| Unstable capture | Hold the sensor/surface still, reduce ambient-light changes and retry |
| Lamps flicker over stationary tape/floor | Inspect live voltage variation and calibration separation; improve height/lighting/contrast before tuning thresholds |

A voltage in range does **not** prove successful I2C communication: this vendor
interface does not expose an ACK/connection flag here. Repeatable white/black
response on each channel is the meaningful functional check. No second robot
or I2C diagnostic process needs to run alongside the robot executable.

## Saving results for autonomous use

The white and black reference rows remain visible for this program session.
They are **not persisted** across robot-code restarts or reboot and are not
automatically applied to autonomous localization/docking.

Send a screenshot showing all four white references, black references and live
readings, plus the physical channel order and mounting measurements. The measured
arrays can then replace `COBRA_FLOOR_V` and `COBRA_TAPE_V` in `AutonomousConfig.h`.
Only after geometry/order and tape behavior are verified should
`COBRA_CALIBRATED` be enabled. Keep the other autonomous start checks in place.
