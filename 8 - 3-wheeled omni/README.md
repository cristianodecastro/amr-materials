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

Wheel outputs are proportional to the mixed speed requests. There is no 15% boost, individual wheel trim, early-wheel hold, or extra wheel-output ramp. Only common saturation preserves wheel ratios at the electrical cap. CAN duties are rounded to the nearest whole percent. Chassis limits remain half the estimated no-load wheel speed, 90% maximum electrical output, 400 mm/s^2 translation acceleration, and 90 deg/s rotation with 45 deg/s^2 acceleration. Low output requests are no longer boosted past static friction.

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

## Goal navigation: first arena trial (P8-r17)

Use the **Navigation** tab in project 8. Exact starting X/Y are calculated from valid left-IR and rear-sonar readings, with their center offsets added once. This requires manually placing the robot facing arena north and ensuring both beams see the outer walls, including a rear beam passing between delivery objects. It is not automatic localization from an arbitrary unknown pose.

1. Place the center approximately around (250,500) mm facing north (0 degrees), outside the bottom delivery band. Approximate placement is sufficient; keep rear/left outer-wall views clear.
2. Select Autonomous in Studica Control Station and enable it. These actions alone do not start a route.
3. Press **Reference corner** and wait for **Referenced**. Check the calculated X/Y. This button establishes the gyro's arena reference at north without zeroing the physical IMU.
4. Set **Goal X mm**, **Goal Y mm**, and **Goal heading deg**. First suggested goal: **(1600,1600,0)**.
5. Press **Go to goal**. The goal is snapshotted; changing fields affects the next request. **Stop**, Disable, and disconnect stop movement. Re-entering Autonomous requires a new corner reference. After a successful reference, successive goals within that Autonomous session use the current estimated pose.

The map is in `navigation_config::FORBIDDEN` in `src/main/include/RobotConfig.h`. Each row is `(xmin,ymin,xmax,ymax)` in mm. Edit/add rows and redeploy; the count is inferred automatically. Keep-outs represent physical regions without a robot margin. Current interpretations are:

| Region | Bounds |
| --- | --- |
| Upper block: 440 x 780 | (780,1220,1220,2000) |
| Bottom band: 2000 x 220 | (0,0,2000,220) |
| Lower central strip: 30 x 600 | (985,0,1015,600) |

The upper block and strip are centered on X=1000 and attached to their respective outer walls. All floor tape, including the central square, stays traversable.

The conservative rectangular footprint extends 291.74 mm forward (171.74 mm body + 120 mm manipulator), 173 mm rearward (including the sensor face), and 171.74 mm to each side. Full body width is assumed for the manipulator. The initial minimum planning clearance is 40 mm. The 620 mm passage between the strip and upper block admits a north-facing robot center between Y=813 and Y=888.26 mm with that clearance; the planner must account for the projection, not just the wheel-center radius.

A 25 mm, eight-neighbor cost-aware A* search favors clearance. Swept rectangle tests check complete segments, so a diagonal cannot cut an obstacle corner or skip a thin wall. Clearance-aware shortcuts remove grid zigzags. XY travel holds its starting heading; the final rotation is checked over its swept outline before a route is accepted, and again before turning. An unreachable start/goal or unsafe final rotation is reported with zero motor output. This is a first stop-and-go trajectory implementation, not a continuous spline or time-optimal controller.

Navigation speed is capped at 180 mm/s with 200 mm/s^2 acceleration. The shared waypoint controller brakes predictively and allows concurrent gyro heading correction. Intermediate observations are no farther than 300 mm apart along a segment. At each stop, 0.65 seconds of fresh readings are considered. Correction requires at least two stable coordinate estimates (spread <=20 mm), near-normal views, consistent predicted range (residual <=60 mm), and no keep-out crossed by the central or +/-15 degree association rays. That angular guard is conservative and is not a claimed sensor beam specification.

Readings through storage/delivery regions never correct position during a route. Missing, invalid, stale, repeated, or ambiguous readings are skipped. A waypoint can proceed on encoder/gyro odometry if no usable wall is visible. Stable accepted wall estimates correct only X/Y, preserve gyro and encoder baselines, and trigger replanning from the corrected position. Wall observations occur while stopped, avoiding stale moving-range corrections. Accelerometer data are not double-integrated into autonomous position; existing Teleop observational telemetry remains separate.

Planning runs in a worker so it cannot block the 20 ms control loop. A canceled/old planning result cannot resume motion or overwrite a newer goal. Tape tests and navigation are mutually exclusive; Teleop and on-demand camera capture are retained. No motor trims, static-friction boosts, or empirical odometry scaling are reintroduced.

The yaw direction monitor allows 300 ms after a yaw request starts or reverses, then stops after five degrees of sustained opposite rotation. Braking/coasting during that initial response interval is not counted as a wiring fault. The Navigation tab shows **Yaw request deg/s** and **Gyro yaw deg/s**; positive is CCW. On a fault, these retain the feedback request and measured rate which triggered it. Revision P8-r17 retains the common motor polarity correction based on the reversed physical Teleop directions and corrects the encoder polarity using the clarified observation that CW rolling increases raw counts. After deployment, confirm stick-forward moves forward and right-stick-left turns CCW before autonomous navigation.

Validation includes nominal and non-grid-aligned starts, thin-wall and diagonal clearance, unsafe intermediate rotations, north-zero gyro/odometry transforms, invalid-range rejection, preservation of encoder/gyro baselines during XY correction, and four motor-lag navigation simulations (including corrupted odometry and unavailable range sensors). Physical arena testing is still required; the 10 mm tolerance applies to the estimated pose and depends on sensor calibration and slip.
