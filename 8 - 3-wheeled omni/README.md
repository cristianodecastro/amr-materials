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

## Autonomous routes and sensor support (P8-r25 MVP)

Navigation follows an editable sequence of intermediate **poses**, then the final pose. Encoders and the reset-relative gyro update motion every 20 ms. Suitable wall readings provide ongoing XY correction, and known barriers or new range detections can cause automatic bypass poses to be inserted. A transit guide blocked by confirmed obstacle evidence may be bypassed; the remaining requested order and original final goal are preserved.

### Initialize the arena pose

1. Place the robot outside the red structures in `docs/court_barriers.png`. Know its heading and ensure that the sensors you select see the surfaces you identify, rather than a storage or delivery object in front of them.
2. Select and enable Autonomous in Control Station. This alone does not start travel.
3. Open **Initialization**. Enter a number or `-` in **Start X mm or -** and **Start Y mm or -**. A dash (or empty field) means that sensors must determine that coordinate. You may supply neither, one, or both coordinates. Enter **Start heading deg** explicitly: 0 north, +90 west, -90 east.
4. Select reference sensors through their wall text fields. `-` disables that sensor for initialization. `X=0` identifies a vertical surface at arena X=0; `Y=2000` identifies a horizontal surface at arena Y=2000. Lowercase x/y also work. The coordinate is the surface's position in the arena, not its distance from the robot.
5. Press **Initialize pose** on Initialization or Navigation. Settings are captured at that press. Supplied X/Y coordinates remain as entered during initialization. Unknown coordinates are calculated from valid, fresh selected measurements, sensor mounting offsets and the supplied heading. Multiple readings of the same axis are averaged with squared beam-axis projection weights. There is no covariance model.
6. Check **Initialized**, **Initialized X/Y/heading**, and the source summary. If an unspecified axis cannot be determined, the robot remains stopped and identifies that axis; correct the selection/reading or supply that coordinate and press Initialize again. There is no stationary gate or reference timeout. Initialization does not start travel.

For the usual lower-left, north-facing reference:

| Field | Entry | Meaning |
| --- | --- | --- |
| Start X / Start Y | `-` / `-` | Derive both coordinates |
| Start heading | `0` | Front points north |
| Left IR wall | `X=0` | X = left distance + 60 mm |
| Rear sonar wall | `Y=0` | Y = rear distance + 173 mm |
| Right IR / Front sonar wall | `-` / `-` | Not used to initialize |

For example, X=`250`, Y=`-`, rear=`Y=0`, other sensors=`-` keeps initial X=250 and measures initial Y. Supplying both X/Y allows initialization without a range reading. Starting heading is still required: scalar ranges alone cannot establish all of X, Y and yaw. The gyro is aligned to this heading using its current reading as an offset, never as a raw arena angle.

For any selected sensor, the solved axis is `wall coordinate - rotated mounting offset - measured range * beam-axis component`. Invalid, stale, nonfinite or out-of-range readings are ignored. Initialization also skips beams whose absolute axis component is below 0.5 and inferred centers outside the arena.

### Choose intermediate poses and run

1. On **Navigation**, select **Saved goal** or **Custom**, then edit final X/Y/heading.
2. In **Intermediate poses (x,y,deg; ...)**, enter poses in order, for example `250,700,0;500,900,-90`. Coordinates are mm and headings degrees. A blank list goes directly toward the final goal. The final goal is appended automatically. Up to 32 user intermediates, whitespace and a trailing semicolon are accepted.
3. Old `x,y` entries still work: they inherit the preceding heading, or the heading at Go for the first entry. Explicit triples are recommended.
4. Press **Go to goal**. The full list and final pose are captured once. Edits during a run apply to the next run.
5. Ordinary intermediate poses are **pass-through guides**, not precision stops. They advance within **25 mm**, or after crossing the approach plane within a 25 mm cross-track corridor. Heading has a relaxed **10 degree** moving deadband; a late heading keeps correcting during translation after the guide is passed. It no longer holds XY motion at an ordinary intermediate. Only the final goal requires the complete 10 mm / 1 degree pose tolerance. Explicit in-place turn entries still finish their requested turn, and collision-critical alignment can require tighter orientation before entering a constrained passage.
6. The final leg holds the preceding travel heading. Final heading is applied after final XY arrival, retaining **10 mm / 1 degree** completion tolerances and a stationary settling check. Live wall corrections continue through final approach and settling. A recent accepted wall residual above 5 mm postpones completion while it converges; final XY drift is corrected without replaying completed requested poses.
7. **Stop**, Disable and disconnect stop motors. A new Autonomous session requires initialization again.

Each saved goal retains its own edited final pose and intermediate list while switching selections, until program restart. Durable defaults are in `navigation_config::GOALS` in `RobotConfig.h`: name, final X, final Y, final heading degrees, intermediate pose string. Add/remove entries there as needed. Initialization and route edits are separate snapshots.

The **Top right** preset ends at **(1700,1700,90 degrees)**. Its editable guides are `650,850,0;1500,850,90;1700,1400,90`, retaining the user's earlier 90 degree turn at the second guide. The 90 degree approach turns the front west before the final northward displacement, keeping its front projection away from the top wall. These are starting route values for the configured map, not a replacement for checking the actual barrier bounds.

Intermediate tracking uses a **100 mm lookahead**, blends up to **25 mm** onto the next segment when that connection is clear, and preserves the velocity ramp across a waypoint change. Straight guides no longer cause a proportional slowdown toward 120 mm/s at each point. Corners reduce speed according to their direction change; final braking is anticipated across the last intermediate. Translation acceleration remains 300 mm/s^2. Heading corrections and XY motion remain simultaneous wherever the footprint allows it.

### Wall measurements during motion

All four sensors are eligible to support travel, independently of which sensors initialized the pose. The four outer arena walls are known references. Additional surface coordinates entered at initialization are also retained; use long, known planar surfaces for those additional references. They are modeled as full planes, not bounded storage faces. After a turn, whichever sensor faces a known surface can supply that axis.

For each new reading, the controller predicts the nearest positive ray/plane intersection. It accepts a correction only when the reading is valid/fresh, the beam is within **15 degrees of the wall normal**, gyro rate is at most **5 degrees/s**, and measured versus predicted range differs by at most **100 mm**. Large mismatches, including most returns from objects in front of the wall, do not change odometry. Readings are processed once per sensor timestamp; a cached value is not repeatedly fused at the 50 Hz control rate.

A 0.5-second pose history aligns a reading with the pose when it was acquired. Sonar timestamps now refer to the ping, rather than the later counter read. Accepted axis estimates are combined geometrically, blended by **25%**, and limited to **5 mm per control update per axis**. Corrections shift XY only; encoder baselines, gyro offset, wheel geometry, counts-per-revolution and motor outputs are not rescaled. **Travel wall support** reports whether recent wall corrections exist for each axis. Where wall support is unavailable, encoders and gyro continue; invalid measurements never become zero-distance corrections.

These checks reduce false association but cannot prove that an echo came from the intended wall. A nearby object within the 100 mm acceptance band may still bias an estimate. Likewise, a 10 mm controller tolerance is not a guarantee of 10 mm physical accuracy from uncalibrated IR/ultrasonic measurements. Check the live distances against the arena before relying on that accuracy.

### Automatic detours

The fixed map uses the supplied 2000 x 2000 arena and previously specified region dimensions. The editable `navigation_config::COURT_BLOCKS` table contains:

| Region | xmin | ymin | xmax | ymax |
| --- | ---: | ---: | ---: | ---: |
| Upper storage block | 780 | 1220 | 1220 | 2000 |
| Bottom delivery band | 0 | 0 | 2000 | 220 |
| Lower central divider | 985 | 0 | 1015 | 600 |

Confirm these bounds against the physical court; the red sketch is not a precisely dimensioned outline. The central tape square and other tape markings remain traversable. Add rows for further fixed structures.

The robot envelope is a **171.74 mm body radius** plus a full-body-width front projection reaching **291.74 mm** from center. Collision checks include that projection, arena boundaries, known blocks and a nominal **30 mm clearance**. Grid detour corners add another 25 mm for intermediate passing tolerance. A start inside the clearance buffer can move monotonically away from it; it is not rejected merely for lacking the full buffer.

When a leg is obstructed, a bounded **50 mm grid A*** search calculates a bypass. If the coarse grid and 25 mm tracking buffer reject a passage, a **25 mm grid with 10 mm extra tracking padding** is tried. Robot dimensions and the 30 mm physical clearance are retained. At a narrowly placed guide, the passing radius and corner blend are reduced to the available clearance, down to 5 mm; crossing-plane advancement still avoids a precision stop. Normal bypasses run at **200 mm/s**. There is no routine wait for the obstacle to disappear or wall-reference pause at intermediate points.

A failed fixed-heading search no longer immediately ends the task. Recovery tries the requested pose heading and other orientations, including clear nearby staging locations when the front projection cannot safely swing where the robot currently is. Where a short path prefix admits the complete turning sweep, the heading changes during translation. A guide now occupied by confirmed obstacle evidence can be bypassed in favor of a later guide; the final goal is preserved. Remaining temporary poses are replaced when necessary. If a complete connection is still unavailable, the controller can move to a clearer nearby pose with both map clearance and fresh sensor evidence, then retry. Recovery movement uses a **120 mm/s request**, subject to the unchanged 15% floor. It avoids immediately returning to the previous retreat origin.

Fresh unexplained range returns within **650 mm** can add obstacle patches after **two consistent readings from the same sensor within 0.8 seconds**, with endpoints within 100 mm. A cached reading cannot confirm itself. An urgent return inside the estimated stopping clearance takes effect immediately. Known court structures and known-wall returns retain their existing geometry. For an otherwise unexplained return, the default assumed obstacle is **200 mm wide and 200 mm deep**, extending behind its detected near face. Change `SENSED_OBSTACLE_RADIUS_MM` to adjust this assumption. Adjacent observations add coverage of longer walls. Up to 24 confirmed patches are retained; **two fresh rays that pass fully through an old patch and at least 60 mm beyond it remove that disproved patch**. Long readings and known-wall readings also provide this free-space evidence. Unseen confirmed obstacles are not silently timed out. Patches are cleared at the next Go; fixed court blocks remain. Patches and provisional hit coordinates follow any estimator-frame XY correction.

Turns check the manipulator sweep. When rotation lacks room, translation can continue without that turn until room exists; if needed, temporary staging poses move the robot to a clear location, turn, and return to the requested pose. Automatic staging turns use a tighter 1 degree tolerance **only if the outgoing segment is blocked at the current orientation**; otherwise they use the relaxed 10 degree tolerance. A small heading discrepancy no longer inserts an extra stop-and-align pose when it can be corrected during translation. A short coasting prediction removes only the commanded component that consumes clearance, preserving tangential/away motion and releasing turning power when necessary. It does not enter a wait-for-clearance state.

Four narrow range beams cannot detect every obstacle edge or establish a long wall's complete extent. The fixed map supplies the known wall/storage/delivery geometry; live detections supplement it. If neither a full recovery route nor a sensor/map-verified movement exists, the controller reports **Recovery blocked** and stops rather than driving blindly. Disable, disconnect, hardware faults and Stop remain effective. A literal never-stop guarantee cannot hold for an enclosed or physically unreachable goal. Required sensor coverage, exact map bounds, obstacle-size assumptions and actual stopping behavior still need arena verification.

### Limits and implementation

Normal requested translation is **350 mm/s**, acceleration **300 mm/s^2**; bypasses use **200 mm/s** and recovery requests **120 mm/s**. Travel heading gain remains 1.2, normally with a 3 degree deadband on the final leg and 10 degrees on ordinary intermediates. The 15% electrical floor and bounded common scaling remain; no individual motor trim is introduced. Essential stops remain for invalid control data/IMU/odometry, a real drive/encoder fault, disconnect/Disable, exhausted verified recovery choices, and a 60-second task deadline. Teleop controls, Teleop pose observation, camera capture, project 7 and build/vendor configuration are unchanged.

`Navigation.cpp` handles routes, initialization and live sensor support. `LocalNavigation.cpp` contains the small footprint/map/grid-detour implementation. Shared heading/position and tape control stay in `Autonomous.cpp`. Defaults, geometry, tolerances, sensor acceptance thresholds and obstacle assumptions are in `RobotConfig.h`. Initialization and Navigation each fit an 8-column, 5-row dashboard; Motion and Sensing keep their compact layouts.

Validation includes twenty-eight motion simulations using encoder odometry, gyro feedback, the actual electrical floor and motor lag. Additional recovery cases start at the reported **(632.8,580.8,-4.37 degrees)** pose, both with clear sensing and a simulated confirmed obstacle blocking the first guide, and reach the original top-right goal. Direct top-right travel without intermediates also tests automatic orientation changes. These run at two motor response times and check mapped/detected footprint collisions. Separate checks cover obstacle confirmation, cached/stale evidence rejection, removal after contradictory rays, urgent returns, sensor-verified retreat, continuous transit, initialization, wall fusion, parser/snapshot/Stop behavior, tape and Teleop. These software cases reproduce planner limitations; the original robot's exact failed sensor map was not recorded. The project has not been deployed or physically validated by this change.
