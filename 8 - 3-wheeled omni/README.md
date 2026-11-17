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

Body coordinates are +X forward, +Y left; positive rotation is counterclockwise. The picture instead draws +Y toward the robot front. Picture angles 150/270/30 degrees therefore become body angles 60/180/300 degrees. Arena coordinates remain +X east and +Y north. Heading zero is NORTH (+Y), +90 degrees is west, -90 degrees is east, and +/-180 degrees is south. Tape tests start north; Navigation accepts a supplied starting heading. Arena yaw is converted to Cartesian angles only for trigonometry; raw gyro yaw remains a reset-relative reference.

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

## Sensor-supported navigation (P8-r28 MVP)

Navigation follows your editable intermediate poses and final goal. Encoder counts and reset-relative gyro heading remain the basic motion estimate. Valid range readings provide a small position offset to guide driving; they do not veto a route or generate recovery faults. The obstacle map, grid searches, automatic detours, staging/recovery controller and planning worker have been deleted.

### Initialize

1. Select and enable Autonomous in Control Station. This alone does not start driving.
2. On **Initialization**, enter the known start X/Y in millimeters. Use `-` for either unknown coordinate, and enter the starting heading explicitly.
3. In each sensor's wall field, use `X=coordinate` for a vertical wall, `Y=coordinate` for a horizontal wall, or `-` to exclude that sensor from initialization. These are global wall coordinates, not measured gaps.
4. Press **Initialize pose**. Supplied coordinates stay as entered; valid selected readings determine the unspecified axes. If an axis cannot be determined, supply it or correct the reference selection. There is no stationary gate or reference timeout.

The usual north-facing lower-left reference is X=`-`, Y=`-`, heading=`0`, left IR=`X=0`, rear sonar=`Y=0`, right/front=`-`. This gives X = left range + 60 mm, Y = rear range + 173 mm. Both coordinates may instead be supplied directly. Initialization establishes the gyro offset using its current reading; raw gyro yaw is not an absolute arena heading.

### Route and controls

On **Navigation**, choose **Saved goal** or **Custom**, edit final X/Y/heading and enter zero or more **Intermediate poses** as `x,y,heading; x,y,heading`. Empty means a direct leg. Old `x,y` entries inherit the preceding heading. Up to 32 intermediate poses are supported. All distances are millimeters; headings are degrees.

Heading zero is north (+Y); +90 is west, -90 east and +/-180 south. The **Top right** preset remains `(1700,1700,90)` with guides `650,850,0;1500,850,90;1700,1400,90`. Each saved goal keeps its own edits while switching selections until restart; durable defaults are in `navigation_config::GOALS` in `RobotConfig.h`.

Press **Go to goal** to snapshot the selected route. Intermediates use a 25 mm passing radius/cross-track corridor and relaxed 10 degree heading guidance. A 100 mm lookahead, up to 25 mm corner blending and a continuous acceleration ramp smooth transitions. An explicitly requested in-place turn still rotates there. The final XY arrival is followed by the final heading and the normal **10 mm / 1 degree** stationary completion check.

Every control update recalculates the remaining segment, lookahead direction, distance and velocity request from the latest sensor-assisted driving pose. A valid range discrepancy can therefore change the robot's path to the next fixed waypoint without stopping, resetting the route or changing the final goal. Final correction pulses also update their direction when the estimated position changes.

During transit, heading correction starts only above **12 degrees** of error and releases near **6 degrees**, with a lower gain of **0.8**. This hysteresis avoids repeated small gyro corrections at one threshold. Inside the last **200 mm of route distance**, heading hold tightens to **3 degrees**, gain **1.2**. Planned large turns and translation remain simultaneous; final stopped heading retains the 1 degree tolerance. Sensor-based global XY correction is still committed only during the stopped final settle.

Normal requested speed is **350 mm/s**, acceleration **300 mm/s^2**. The existing 15% nonzero motor-output floor remains. The route is defined by your intermediate poses: no automatic obstacle detour or collision-veto layer remains. Place the poses so the segments, corner blends and turns have room for the robot and manipulator.

### Motion tuning in one header

Edit **`src/main/include/RobotConfig.h`** for all motion tuning. The index at its top identifies each section. Angles intended for tuning are specified in degrees; their radian constants are derived automatically.

| Section | Main settings |
| --- | --- |
| `navigation_config` | Translation speed/acceleration; 25 mm intermediate and 10 mm final tolerances; lookahead/corner blend; 12/6 degree transit hysteresis; transit gain; 200 mm final approach distance; approach heading gain/deadband; final settle duration; range blend/step limits |
| `heading_control_config` | Final 1 degree tolerance; yaw speed/acceleration; rate damping; coasting prediction; heading recheck; stationary thresholds |
| `position_control_config` | Shared final XY proportional gain; minimum approach speed; braking response; capture-radius ratio; velocity filter; correction/rest/settle times |
| `wheel_drive_config` | PWM floor; bounded common output scaling; neutral thresholds; encoder speed, stall and direction thresholds |
| `localization_config` | Wheel geometry/count conversion; sensor mounts; control/sample timing; Teleop speeds, acceleration and stick deadband |
| `tape_test_config` | Tape approach targets, speed/acceleration, scan/centering settings and task limits |

The shared final XY controller now reads `position_control_config`, so its tuning is no longer hidden among tape-only settings. Moving existing Teleop values into named constants preserves their previous defaults. No motion gain, speed limit or tolerance needs a source-file edit.

### Sensor geometry and driving feedback

This first version uses the four known outer arena walls plus the additional reference planes entered at initialization. It is sensor-assisted localization against known coordinates; it does not learn unknown obstacles as a SLAM system.

Sensor mounts in the body frame (+X forward, +Y left) remain:

| Sensor | Body offset (forward, left), mm | Beam angle from body-forward |
| --- | --- | --- |
| Left IR | (0, 60) | +90 degrees |
| Rear sonar | (-173, 0) | 180 degrees |
| Right IR | (0, -60) | -90 degrees |
| Front sonar | (171.6, 0) | 0 degrees |

Every beam stays perpendicular to its robot side. For arena heading `h`, the Cartesian rotation is `h + 90 degrees`. Both the mount offset and beam direction are rotated with that angle. For a known plane X=Xw, the inferred center is `Xw - rotated_mount_x - range * beam_x`; Y uses the corresponding Y components. Multiple readings of one axis use squared beam-component weights.

A short pose history supplies the heading/position at acquisition time, including the sonar ping time. Oblique readings and readings during rotation can contribute; there is no near-normal-heading or low-yaw-rate requirement. Exactly parallel beams cannot observe the perpendicular axis.

A reading must have a valid flag, finite in-range value and fresh timestamp. IR validity uses the existing final doubled distance (100-800 mm); sonar validity is 20-4000 mm. Cached readings are consumed once. A reading more than 100 mm from the predicted known-wall range is not associated with that wall and is silently skipped; a valid return from an unrelated object must not redefine the global position. None of these range checks stops motion.

While driving, the accepted sensor/encoder discrepancy updates a separate offset, blended by 25% and limited to 5 mm per new update per axis. The controller uses **encoder pose + range offset**. For example, at heading 0 a rear reading of 277 mm plus its 173 mm mount offset indicates center Y=450 mm. If encoder Y is 500 mm, driving feedback gradually incorporates the -50 mm difference. Encoder distance, wheel calibration, gyro reference and global odometry are not rewritten during travel.

On Navigation, **Driving X/Y** shows the sensor-assisted pose used for control. Motion's X/Y remains the global odometry pose. **Travel wall support** indicates recent accepted readings for each axis. When readings are missing or invalid, the route continues from encoders/gyro and the last accepted driving offset; invalid values never become zero-distance corrections.

### Stopped goal update

After the normal goal pose/heading check and a 0.3-second stationary settle, only range estimates acquired during that settle and still supported by fresh valid sensor data may be committed to global XY odometry. There is no extra sensor-residual gate and no requirement that every sensor be available. If the resulting position is outside the 10 mm tolerance, the normal position controller makes the final correction. Otherwise the goal completes. Unavailable ranges are skipped, not treated as task failures.

The same valid selected ranges may establish global X/Y during explicit initialization before travel. No covariance machinery, motor trims or empirical odometry scales are added.

### Remaining stops and validation

The retained navigation protections are explicit Stop/Disable/disconnect, unavailable or invalid gyro/odometry/control data, and the existing hardware motor/encoder fault checks. Malformed/nonfinite settings are rejected before starting. There is no navigation task deadline, progress watchdog, scene-feasibility error, recovery failure or sensor-required travel gate.

The bounded late-sample handling remains: autonomous odometry preserves encoder travel through intervals up to 500 ms, while controller ramps use at most a 40 ms step. Larger/unusable feedback gaps remain control-data faults. Teleop timing and behavior, camera capture, project 7 and build/vendor configuration are unchanged. The compact **Loop / peak ms** tile remains; planner diagnostics are removed with the planner.

`Navigation.cpp` now contains route following, reference initialization and range support. `Autonomous.cpp` retains the shared heading/position and tape controllers. `LocalNavigation.cpp` and its planner/map classes are deleted. Existing component-check source covers beam/mount projections, invalid readings, stopped correction, control transitions and timing; motion-model simulation code has been removed. No simulations are run after the user's prohibition. Validation for this revision is compilation/build checks; physical behavior must be evaluated on the robot.
