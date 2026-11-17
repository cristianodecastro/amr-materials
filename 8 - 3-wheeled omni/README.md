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

## Goal navigation: first arena trial (P8-r21)

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

A 25 mm, eight-neighbor cost-aware A* search favors clearance. Swept rectangle tests check complete segments, so a diagonal cannot cut an obstacle corner or skip a thin wall. Clearance-aware shortcuts remove grid zigzags. **Preferred travel heading is 0 degrees (north). There is no fixed -90 degree crossing angle and no fixed turn pocket.** The final requested pose remains editable on Shuffleboard.

Navigation follows the planner's vertices continuously, using a 140 mm lookahead and a 60 mm passing radius. A guide may also be passed after crossing its approach plane, provided the next segment is clear at the selected heading. These are steering settings, not intermediate arrival tolerances. The local motion guard checks the actual heading while the robot turns, so future-path checks do not require it to finish rotating before translating. If a heading change needs a different centreline through the gap, the route is replanned rather than chasing a guide already behind the robot. Ordinary intermediate guides have no precision stop or wall-reading pause.

When front clearance becomes relevant, the controller searches in **2 degree increments** for the smallest feasible deviation from north. Route direction breaks ties between left and right. The full rectangle, including the 120 mm manipulator projection, is checked at the preview and over the entire turn sweep. A turn can initially increase front reach, so a cosine-only calculation using just the manipulator length would be wrong. The usual desired front gap is **100 mm**, with 20 mm return hysteresis. If that soft gap cannot fit, it is reduced in 10 mm steps while retaining the **40 mm geometric planning margin**. The controller returns toward 0 degrees when the measured/map geometry permits the complete return sweep.

Known frontal surfaces enter this check within 300 mm of the front footprint, including lateral padding of half the body width. This covers manipulator corners outside one centre sonar beam. Fresh front sonar is the primary live evidence; side IR and rear sonar corroborate nearby obstacles as the robot rotates. The measured sensor-face range is projected from its actual mount, so front-sonar clearance accounts for **291.74 - 171.6 = 120.14 mm** of manipulator reach beyond that sensor. Distances under 650 mm may add local obstacle evidence; invalid, stale or repeated samples do not. Sensor pose is interpolated from a one-second history. Sonar acquisition timestamps use the ping time, not the counter-read time 60 ms later.

An unexplained first reading constrains motion temporarily. A second consistent distinct reading confirms a static obstacle; an isolated outlier expires after the normal freshness interval. Confirmed obstacles stay in arena coordinates when the beam turns away, rather than disappearing and causing an immediate turn back toward the wall. There are at most 12 retained patches, replacing the farthest if necessary. Hits near a known horizontal face use that face's finite span with body-width edge padding and a 10 mm range allowance; other unexplained hits are finite uncertain patches, not inferred infinite walls. These uncertainty allowances are tuning choices, not sensor field-of-view specifications. Re-referencing the corner clears this temporary obstacle map.

The local guard predicts the **next 20 ms control interval plus braking**, including CAN quantization, the 15% floor/common scaling, current encoder velocity and gyro rate. Each candidate must pass both 120 ms and 180 ms command-response models, with a 180 ms braking-response model. It checks full swept footprints and may reduce translation, keep only a safe component, turn while translating, brake, or back away to restore a newly observed clearance margin. If new evidence reveals that the robot is already inside the planning margin, checked recovery motion must improve the predicted clearance before replanning. These models and clearances are editable in `navigation_config`; their assumptions still need a physical arena trial. If confirmed evidence invalidates the old route, A* replans using it, and an asynchronous result is revalidated before movement. A distant segment temporarily not fitting the selected heading does not itself force a stop when a safe local preview remains available.

If a fixed-heading route cannot fit, the worker searches feasible headings in 5 degree increments. If necessary it also searches nearby relative displacements of 25/50/75 mm with checked turns before and after that displacement. Thus exceptional recovery turns/locations are selected from current geometry, not a prescribed east-facing route. Recovery repositions use a 15 mm stopping tolerance and recheck the actual rotation sweep before turning. A physically blocked route is stopped rather than blindly rotating.

Travel heading correction retains gain 1.2 and a 3 degree deadband where clearance permits; it tightens where the swept footprint needs it. Translation and yaw correction run together. Requested cruise remains **300 mm/s**, with **300 mm/s^2** acceleration, **220 mm/s** near obstacles, and a **120 mm/s** base corner speed. Teleop speeds are unchanged.

Final approach starts within 280 mm with a 160 mm/s approach-speed cap and preserves the previous velocity command. Final arrival still requires **10 mm / 1 degree** and settled translation/rotation. The preferred 100 mm front gap becomes the hard planning margin on final approach so an otherwise safe goal closer to a wall remains reachable.

Wall referencing now occurs at the destination, after motion settles, rather than pausing at each guide. During the 0.65 second reading window, outputs remain zero and the measured heading is used for wall association; there is no extra turn to an intermediate heading. Correction requires at least two stable coordinate estimates (spread <=20 mm), near-normal views, consistent predicted range (residual <=60 mm), and no keep-out crossed by the central or +/-15 degree association rays. That angular guard is conservative and is not a claimed sensor beam specification.

Readings through storage/delivery regions never correct position. Missing, invalid, stale, repeated, or ambiguous readings are skipped for localization; navigation can complete on encoder/gyro odometry when no usable wall reference is visible. Stable destination-wall estimates correct only X/Y and preserve gyro and encoder baselines. A correction outside the final 10 mm tolerance triggers a new final approach. Moving readings add obstacle evidence only, never a position reset. Obstacle mapping is suspended during stationary wall localization; after an accepted correction, pose history is reanchored and pre-correction cached samples are discarded. Accelerometer data are not double-integrated into autonomous position.

Planning runs in a worker so it cannot block the 20 ms control loop. A canceled/old planning result cannot resume motion or overwrite a newer goal. Tape tests and navigation are mutually exclusive; Teleop and on-demand camera capture are retained. No individual motor trims or empirical odometry scaling are applied; the same 15% running-duty floor is used for all drive wheels.

The yaw direction monitor allows 300 ms after a yaw request starts or reverses, then stops after five degrees of sustained opposite rotation. Braking/coasting during that initial response interval is not counted as a wiring fault. Navigation shows signed yaw request/gyro rates and status such as **Front clearance: heading ... deg**. Revision P8-r21 retains the speed increase, correct wiring signs and 15% floor, replacing the preset crossing with sensor/map-driven minimum-angle selection. Teleop and camera capture retain their existing behavior.

Validation covers minimum-angle selection, unknown sensor hits, rejected invalid/stale samples, isolated outliers, obstacle memory after turning, acquisition-time pose projection, swept concurrent XY/yaw motion, correct low-speed wheel ratios, and seven motor-lag goal simulations. The tests include 120/180 ms motor response and an uncharted wall 50 mm closer and 160 mm wider than the mapped block; physical footprint clearance and final 10 mm / 1 degree arrival are checked. Sparse fixed beams cannot observe every possible obstacle, so the map, footprint and actual response still matter. Physical arena testing is required before relying on these simulated clearances.
