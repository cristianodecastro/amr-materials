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

## Sensor-supported navigation (P8-r33 MVP)

Navigation follows your editable intermediate poses and final goal. Encoder counts and reset-relative gyro heading remain the basic motion estimate. Valid range readings provide a small position offset to guide driving; they do not veto a route or generate recovery faults. The obstacle map, grid searches, automatic detours, staging/recovery controller and planning worker have been deleted.

### Initialize

1. Connect Control Station and keep the robot **Disabled**. Pose initialization is also available in enabled Autonomous, but never commands driving. Enabled Teleop rejects pose initialization.
2. Place the robot near **any of the five named positions**, not necessarily Start. On **Motion**, select that **Starting area** independently of the destination. The selector loads its heading, sets X/Y to `-` (both must be measured), and fills the appropriate sensor-wall fields. Align the robot to the displayed heading before referencing: the selection supplies a known heading, not a sensor-derived absolute yaw.
3. Ensure each enabled sensor has an unobstructed view of the wall named in its field. Storages/delivery objects are not wall references. Use `X=coordinate` for a vertical wall, `Y=coordinate` for a horizontal wall, or `-` to exclude that sensor. These are global wall coordinates, not measured gaps. If you change the heading, review which sensor sees which wall.
4. Press **Initialize pose**. Valid, finite, fresh, in-range selected readings establish actual X/Y, accounting for rotated sensor offsets and beam directions. Named areas always measure both axes when the button is pressed, even if the numeric X/Y fields had been edited; select Manual to supply coordinates. Nominal named X/Y are never copied into odometry by this mode. If an axis cannot be determined, initialization waits stopped for a valid reference; it does not drive from an assumed coordinate. There is no stationary gate or reference timeout.
5. Watch **Pose reference** for the sensed X/Y or the reason it is waiting. **Referenced** becomes true and Motion's existing **X/Y/Heading** tiles show the updated global pose. Then select and enable Autonomous, choose the separate **Saved goal** on **Navigation**, and press **Go to goal**. Enabling Autonomous preserves the completed reference; it does not reset it. The first route uses the selected starting area, even when the measured pose is more than 75 mm from its nominal coordinates.

Motion now contains initialization in an 8-column, 5-row grid; the separate Initialization tab and duplicate navigation Initialize button are removed. Motor counts/speeds/PWM and initialization fields use single-cell tiles, while global pose, starting-area selection and status fields have more width. Duplicate initialized-pose displays are removed; use the existing global X/Y/Heading tiles. A saved desktop Shuffleboard layout can retain an obsolete Initialization tab; close that local tab after deploying the new program.

Default starting-area profiles (`navigation_config::INITIAL_WALLS`, sensor order left IR / rear sonar / right IR / front sonar) are:

| Starting area | Heading | Enabled reference sensors |
| --- | --- | --- |
| Storage right | 90 | Rear sonar: X=2000; right IR: Y=2000 |
| Storage left | -90 | Rear sonar: X=0; left IR: Y=2000 |
| Delivery right | 90 | Rear sonar: X=2000; left IR: Y=0 |
| Delivery left | 0 | Rear sonar: Y=0; left IR: X=0 |
| Start | -90 | Rear sonar: X=0; front sonar: X=2000; left IR: horizontal wall Y=1200 |

Changing Starting area while idle clears the old reference and requires Initialize again. During travel, a changed selection is deferred until motion completes; it never resets an active route. Switching destinations alone does not change the starting-area selection or require reinitialization after each arrival. To physically relocate the robot near another area, select that area and initialize again; if it is the same selection, press Initialize again.

**Manual** keeps the editable fields, allowing supplied X/Y values or `-` for unknown axes. Only explicitly supplied coordinates are taken as known. Valid selected readings determine the unspecified axes. Changing area does not change the saved goal coordinates or the active gyro until explicit initialization succeeds.

The usual north-facing lower-left reference is X=`-`, Y=`-`, heading=`0`, left IR=`X=0`, rear sonar=`Y=0`, right/front=`-`. This gives X = left range + 60 mm, Y = rear range + 173 mm. Both coordinates may instead be supplied directly. Initialization establishes the gyro offset using its current reading; raw gyro yaw is not an absolute arena heading.

### Route and controls

On **Navigation**, choose **Saved goal** or **Custom**, edit final X/Y/heading and enter zero or more **Intermediate poses** as `x,y,heading; x,y,heading`. Empty means a direct leg. Old `x,y` entries inherit the preceding heading. Up to 32 intermediate poses are supported. All distances are millimeters; headings are degrees.

Heading zero is north (+Y); +90 is west, -90 east and +/-180 south. There are five named poses:

| Saved goal | X (mm) | Y (mm) | Heading (deg) |
| --- | --- | --- | --- |
| Storage right (formerly Top right) | 1700 | 1700 | 90 |
| Storage left | 300 | 1700 | -90 |
| Delivery right | 1700 | 400 | 90 |
| Delivery left | 300 | 400 | 0 |
| Start | 950 | 900 | -90 |

Initialize once at the actual starting pose. After **Goal reached**, choose any of the other four named goals and press **Go to goal** again. Returning to **Start** is ordinary navigation: selecting it does not reset odometry or gyro. Consecutive moves retain the sensor-assisted driving offset on any axis that could not receive a stationary global update at the preceding arrival.

Two shared intermediate poses are defined once in `navigation_config::INTERMEDIATES` in `RobotConfig.h`: **Intermediate left `(500,900,-90)`** and **Intermediate right `(1500,900,-90)`**. `ROUTES` contains their IDs, rather than repeated coordinates; rows are sources and columns are destinations in the `GOALS` order above.

- Storage and delivery on the same side connect directly, in either direction.
- Transfers between sides use the source-side intermediate, then the destination-side intermediate.
- Transfers between Start and another goal use that goal's side intermediate.
- Selecting the current goal has no intermediates.

For example, Storage left to Delivery right uses Intermediate left, Intermediate right, then Delivery right. Storage left to Delivery left is direct. Delivery right to Start uses Intermediate right, then Start. All intermediate headings are -90 degrees. These remain pass-through poses with relaxed tolerances and simultaneous rotation/translation, without mandatory in-place turns. The dashboard still permits editing, adding or removing intermediate poses per transfer. Physical clearance must be evaluated on the robot.

After named-area initialization, the dashboard uses that area as the first route source. After arrival, it identifies the nearest named pose within 75 mm and loads the next source-to-destination list; **Navigation status** shows `from <source>`. The source stays fixed during travel. For manual initialization with no nearby named source, it shows **Current pose** and uses a separate, initially empty guide list. Review/edit that list before Go when starting elsewhere. Custom destinations also have separate guide lists.

Final X/Y/heading edits belong to the selected destination. Intermediate-list edits belong to the current source–destination pair. Both survive switching selections until the program restarts; durable defaults are in `navigation_config::GOALS` and `ROUTES` in `RobotConfig.h`. Changing a selection or editing a field during travel does not change the active captured route.

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

The updated drawing's written restricted-region dimensions are recorded in `arena_config`: upper 230 x 780 mm, bottom band 2000 x 230 mm, and lower central strip 30 x 370 mm. The divider centreline is X=1000 mm; the lower strip spans X=985..1015 mm. Start remains at X=950 mm, independently of that centreline, accounting for the manipulator. These are scene descriptions, not new route vetoes or obstacles inferred from floor tape.

The horizontal wooden wall below the upper region is separate from that restricted region. Its finite reference uses X=700..1300 mm (600 mm centred on X=1000), with faces Y=1200/1220 mm. These remain editable estimates based on the drawing and the user's approximate confirmation. `HORIZONTAL_WALL_REFERENCE_ENABLED` is now **true**. Only beams that intersect its actual span can reference it; beams passing its ends can still reference outer walls. The usual valid/fresh-reading and discrepancy checks apply, including sensor mount offsets and acquisition-time heading. This adds no sensor-required stop or clearance failure.

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

On Navigation, **Driving X/Y** shows the sensor-assisted pose used for control. Motion's X/Y remains the global odometry pose. When readings are missing or invalid, the route continues from encoders/gyro and the last accepted driving offset; invalid values never become zero-distance corrections.

### Stopped goal update

After the normal goal pose/heading check and a 0.3-second stationary settle, only range estimates acquired during that settle and still supported by fresh valid sensor data may be committed to global XY odometry. There is no extra sensor-residual gate and no requirement that every sensor be available. If the resulting position is outside the 10 mm tolerance, the normal position controller makes the final correction. Otherwise the goal completes. Unavailable ranges are skipped, not treated as task failures.

The same valid selected ranges may establish global X/Y during explicit initialization before travel. No covariance machinery, motor trims or empirical odometry scales are added.

### Remaining stops and validation

The retained navigation protections are explicit Stop/Disable/disconnect, unavailable or invalid gyro/odometry/control data, and the existing hardware motor/encoder fault checks. Malformed/nonfinite settings are rejected before starting. There is no navigation task deadline, progress watchdog, scene-feasibility error, recovery failure or sensor-required travel gate.

The bounded late-sample handling remains: autonomous odometry preserves encoder travel through intervals up to 500 ms, while controller ramps use at most a 40 ms step. Larger/unusable feedback gaps remain control-data faults during travel. Sensor initialization can recover an old odometry health fault by resetting it, while retaining IMU/Control Station/clock checks. Old drive faults do not prevent a sensor-only reference; they remain checked before driving. Teleop driving behavior, camera capture, project 7 and build/vendor configuration are unchanged. The Loop/peak timing and duplicate pose/support tiles are omitted from Motion to fit initialization controls.

`Navigation.cpp` now contains route following, reference initialization and range support. `Autonomous.cpp` retains the shared heading/position and tape controllers. `LocalNavigation.cpp` and its planner/map classes are deleted. Existing component-check source covers beam/mount projections, invalid readings, stopped correction, control transitions and timing; motion-model simulation code has been removed. No simulations are run after the user's prohibition. Validation for this revision is compilation/build checks; physical behavior must be evaluated on the robot.
