# Project 8: three-wheel omni robot

Standalone WPILib 2020 / Studica VMX-pi project. Project 7 is a separate reference project and is not modified or required.

## Installed drive

Top view, with robot-forward toward the TOP of the supplied wheel diagram:

| Titan port and encoder | Wheel | Radial position in body frame |
| --- | --- | --- |
| M0 | Front-left / picture upper-left | 60 degrees |
| M1 | Rear / picture bottom | 180 degrees |
| M2 | Front-right / picture upper-right | 300 degrees |

M3 is unused. No lift or servo objects, controls, or dashboard tab are instantiated.

Body coordinates are +X forward, +Y left; positive rotation is counterclockwise. The picture instead draws +Y toward the robot front. Picture angles 150/270/30 degrees therefore become body angles 60/180/300 degrees. Arena coordinates remain +X east and +Y north. Heading zero is now NORTH (+Y), +90 degrees is west, -90 degrees is east, and +/-180 degrees is south. Both corner-reference routines start facing north at 0 degrees. Arena yaw is converted to Cartesian angles only for trigonometry; raw gyro yaw remains a reset-relative reference.

Wheel diameter is 102 mm. The center-to-wheel-center distance L is 159.74 mm for all three tangent wheels. This is the rotational moment arm, not the wheel radius. Encoders use 1464 counts per wheel revolution, giving approximately 0.218881455 mm/count. No external gearing is configured.

For body forward speed f, left speed l, and CCW yaw rate w (rad/s), tangent wheel speeds in mm/s are:

- M0: -sqrt(3)/2 * f + 0.5 * l + L * w
- M1: -l + L * w
- M2: +sqrt(3)/2 * f + 0.5 * l + L * w

Positive canonical wheel speed follows the CCW tangent arrows in the diagram. The installed motors require negative Titan electrical duty for that direction: `WHEEL_MOTOR_SIGN={-1,-1,-1}`. This polarity is applied once at the final hardware output, equally in Teleop and Autonomous. The dashboard PWM shows the signed electrical duty, so its sign can differ from the canonical wheel-speed request. Wiring polarity entries in RobotConfig.h are signs only, not motor trims.

Motor polarity does not determine encoder wiring polarity. The corrected physical observation is that manually rolling the whole chassis CLOCKWISE increases all three raw encoder counts. Therefore `WHEEL_ENCODER_SIGN={-1,-1,-1}` converts them to the CCW-positive wheel convention. This conversion is applied once in autonomous odometry, Teleop pose telemetry and the shared wheel-speed/direction monitor; hardware and dashboard encoder counts remain raw. If wiring changes, repeat the Disabled rolling check: during CCW motion, decreasing raw counts require -1 and increasing raw counts require +1. The gyro remains CCW-positive.

Commands and encoder odometry use the same geometry. Odometry subtracts gyro rotation from each wheel's travel and integrates nominal encoder distance with reset-relative gyro heading. The former 180-degree layout offset, reversed yaw multiplier and empirical 0.9555 odometry scale are removed.

The mixer produces proportional wheel requests with common saturation. At the final Titan output, every nonzero drive command has a **15% minimum electrical duty**, configured by `wheel_drive_config::MIN_RUNNING_OUTPUT`. This applies to translation, rotation and heading correction in Teleop, tape tests and navigation. A deliberately stationary wheel, neutral command, Stop, Disable or fault still produces exactly zero. If an output cap is below 15%, the output is zero rather than exceeding the cap. CAN duties are rounded to whole percentages; the Motion PWM tiles display the actual electrical duty, so they show zero or at least 15% in magnitude. The floor can change wheel-speed ratios at low requests, so it is a breakaway setting, not a calibrated speed guarantee.

Autonomous now applies a bounded common scaling before the final floor. For example, a small lateral request needs wheel magnitudes 15/30/15 to preserve zero yaw; independently flooring all three to 15 changes the chassis direction. The common scaling preserves ratios where possible and never raises an already larger command above its existing peak just to satisfy a nearly cancelled wheel. Its increased peak is bounded to 30% (or the lower electrical cap). The final per-wheel 15% rule still applies. Teleop does not use this new common scaling, and encoder readings never apply motor trims.

There are no individual motor trims, early-wheel holds or extra wheel-output ramps. Chassis limits remain half the estimated no-load wheel speed, 90% maximum electrical output, 400 mm/s^2 translation acceleration, and 90 deg/s rotation with 45 deg/s^2 acceleration. Chassis braking still reaches a true zero, which bypasses the running-duty floor.

## Active controls and dashboard

- Enable Teleoperated mode in Studica Control Station and center both sticks to arm. Left stick translates, including diagonals; right stick left/right rotates CCW/CW. There is no gyro steering in Teleop.
- Disabled, disconnected, and Test modes cannot drive the motors.
- Motion tab: status/revision, enabled/controller/IMU state, pose, loop time, each wheel's encoder/speed/applied PWM, Stop tape, Tape from left, Tape from right.
- Sensing tab: front/rear sonar, left/right IR voltage and distance validity, Cobra CH0..CH3 voltages/detection, and tape scan status.
- Camera tab: press **Take photo** for a fresh USB image. The camera opens only during the request, closes before PNG encoding, and the preview holds the last image (repeated at 1 fps without USB acquisition). Captures retain the 180-degree orientation correction and are saved under `/home/pi/SkillsChallenge/captures`. Copy them to `C:\Users\LENOVO\Pictures\CameraServer` on the PC. Photo capture works independently of drive mode and never enables motion.
- Select and enable Autonomous, then click one tape approach. Both use the existing rear-sonar/left-IR corner reference, then (220,500) or (580,500) mm, then scan inward to center CH1/CH2 on tape. Translation and heading correction may run simultaneously. Completion tolerances are 10 mm and 1 degree.
- Invalid/stale sensor values are not used for referencing. The routine waits stopped for valid reference/Cobra readings and resumes when available; Stop and Disable still abort motion.

Sonars use FlexDIO 2/3 front and 0/1 rear (even trigger, odd echo). IR sensors use VMX Analog In 0 left and 1 right. The requested IR x2 conversion and 60 mm left/right center offsets are retained. Front sonar offset is 171.6 mm; rear sonar offset is 173 mm. Cobra analog J1 remains on VMX I2C; fixed supplied white/black references are authoritative, CH0 leftmost to CH3 rightmost.

Basic teleoperated driving and on-demand photo capture are retained. The inactive project-7 arena mission/planner, simple tasks, colour classification/calibration, Cobra recapture/file persistence, lift, and servo code are removed. Build and vendor configuration remain unchanged.

## Autonomous minimum version (P8-r22 MVP)

The Navigation tab executes an **explicitly supplied sequence of XY points followed by one final pose**. The intermediate list can be empty or contain one or more points. There is no A*, obstacle map, adaptive heading search, recovery search, automatic sensor correction during travel, or background planning job. The old planner preview was removed. Choose the points so every segment, blended corner and the final turn have clear space for the full robot and manipulator.

1. Place the robot facing north (0 degrees), with clear rear-sonar and left-IR views of the outer walls.
2. Select and enable Autonomous in Studica Control Station. This alone does not move the robot.
3. Press **Reference corner**. Valid fresh readings establish X = left range + 60 mm and Y = rear range + 173 mm. The gyro reference is set relative to its current reading. Invalid ranges leave the robot stopped while waiting; there is no stationary gate or reference timeout.
4. Select **Saved goal**. Its final X/Y/heading and its own intermediate list appear in the editable fields. **Custom** is available for another goal. For a first short test, leave the intermediate field blank and set Goal X to current X, Goal Y to current Y + 250 mm, and Goal heading to 0 degrees. The default (250,700,0) is only an example.
5. In **Intermediate XY mm (x,y; x,y)**, enter points in travel order, separated by semicolons. A blank field goes directly to the final goal. For example, `250,700;250,950` means start -> (250,700) -> (250,950) -> the final X/Y/heading. The final goal is added automatically; do not append it to the intermediate list. All values are mm. Up to 32 intermediate points are accepted; whitespace and a trailing semicolon are allowed.
6. Press **Go to goal**. The entire list and final pose are captured once; subsequent edits or another goal selection affect the next run. XY travel holds the starting heading with the gyro, allowing simultaneous translation and heading correction. Intermediate points are passed within 50 mm or after crossing their approach plane, without a precision stop. The speed eases near them and changes smoothly between segments. After the final XY target is reached, the robot sets the requested final angle and settles within **10 mm / 1 degree**.
7. **Stop**, Disable and disconnect stop the motors. A new Autonomous session requires a new corner reference.

Each saved goal retains its own edited final pose and intermediate list when you switch to another goal and back, until the robot program restarts. Durable defaults are the simple `navigation_config::GOALS` table in `RobotConfig.h`: each row contains a name, final X, final Y, final heading in degrees, and its intermediate XY string. Add a row for a new goal, or add/remove `x,y` pairs from its string, then build/deploy. No other code changes are needed. The included north-lane examples demonstrate zero, one and two intermediate points; replace them with your measured routes.

Sensors remain visible on Sensing, but obstacle avoidance is manual in this version. A segment across an arena wall is still a direct command. Account for the 50 mm passing radius and corner blending when placing intermediates. The body outside radius is approximately 171.74 mm, with the manipulator extending another 120 mm at the front; account for that reach when choosing a segment or turn. Intermediate points are XY guides only; the requested final heading is applied after final XY arrival.

The Navigation tab shows status/revision, Reference/Go/Stop, referenced/enabled state, remaining route distance, requested/current pose, saved-goal selection and the intermediate-point editor. Status identifies the current intermediate point. The old path-planner and yaw-diagnostic widgets were removed to keep everything in five rows. Motion, Sensing, Teleop and Camera keep their existing roles.

Requested autonomous speed is **300 mm/s**, acceleration **300 mm/s^2**, with final braking handled by the shared position controller. Heading hold retains gain 1.2 and the relaxed 3 degree travel deadband. The 15% electrical floor and bounded common scaling remain; no individual motor trim is applied. Teleop settings and project 7 are unchanged. All distances are millimeters.

Only essential stops remain: invalid control data or IMU/odometry, disconnected/disabled Control Station, a real drive/encoder fault, and a 60-second task deadline. Invalid intermediate text is rejected before moving. Final XY drift is corrected without replaying completed intermediates. There are no footprint feasibility or route-found conditions that can prevent a valid manually supplied route from starting.

The manual-route controller is in `src/main/cpp/Navigation.cpp`; shared heading/position and tape control are in `Autonomous.cpp`. Speed, travel heading gain, passing radius, timeout and saved goals are together in the small `navigation_config` block in `RobotConfig.h`. Hardware mapping and build/vendor configuration are retained.

Validation includes twelve clear-floor motion simulations with actual encoder odometry, the electrical floor, two motor response times, one-meter north/east translations, both final turn directions, a heading disturbance during XY travel, and routes with zero/one/multiple intermediates. Parser rejection, route snapshots and Stop/reference/fault handling are checked. The existing kinematics, sensor, tape and Teleop checks also pass. These tests verify motion control, not obstacle-free arena routes.
