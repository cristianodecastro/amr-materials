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
- Motion tab: drive diagnostics only: status/revision, enabled/controller/IMU/odometry state, pose, raw gyro/stored heading offset, and one aligned encoder/speed/PWM row per motor. No task-start controls.
- Navigation tab: starting area, wall X/Y, start heading, Initialize, saved destination, goal pose, Go/Stop, current pose, remaining distance, and guides/override. All pose setup and goal controls live here once.
- Tape test tab: left/right approaches and Stop, tape-task status, pose, CH0..CH3 voltages, scan distance, CH1-CH2 difference, and the two corner-reference readings.
- Sensing tab: read-only front/rear sonar, left/right IR voltage and distance validity, and Cobra CH0..CH3 voltages/detection. No tape-task status or commands.
- Camera tab: press **Take photo** for a fresh USB image. The camera opens only during the request, closes before PNG encoding, and the preview holds the last image (repeated at 1 fps without USB acquisition). Captures retain the 180-degree orientation correction and are saved under `/home/pi/SkillsChallenge/captures`. Copy them to `C:\Users\LENOVO\Pictures\CameraServer` on the PC. Photo capture works independently of drive mode and never enables motion.
- On Tape test, select and enable Autonomous, then click one tape approach. Both use the existing rear-sonar/left-IR corner reference, then (220,500) or (580,500) mm, then scan inward to center CH1/CH2 on tape. Translation and heading correction may run simultaneously. Completion tolerances are 10 mm and 1 degree.
- Invalid/stale sensor values are not used for referencing. The routine waits stopped for valid reference/Cobra readings and resumes when available; Stop and Disable still abort motion.

Sonars use FlexDIO 2/3 front and 0/1 rear (even trigger, odd echo). IR sensors use VMX Analog In 0 left and 1 right. The requested IR x2 conversion and 60 mm left/right center offsets are retained. Front sonar offset is 171.6 mm; rear sonar offset is 173 mm. Cobra analog J1 remains on VMX I2C; fixed supplied white/black references are authoritative, CH0 leftmost to CH3 rightmost.

Basic teleoperated driving and on-demand photo capture are retained. The inactive project-7 arena mission/planner, simple tasks, colour classification/calibration, Cobra recapture/file persistence, lift, and servo code are removed. Build and vendor configuration remain unchanged.

## Sensor-supported navigation (P8-r36 MVP)

Navigation follows your editable intermediate poses and final goal. Encoder counts and reset-relative gyro heading remain the basic motion estimate. Valid range readings provide a small position offset to guide driving; they do not veto a route or generate recovery faults. The obstacle map, grid searches, automatic detours, staging/recovery controller and planning worker have been deleted.

### Initialize and select a goal on Navigation

All initialization and goal controls share the **Navigation** tab, within an 8-column, 5-row grid. The old Pose setup/Initialization tabs and fixed per-sensor wall fields are removed. Motion contains drive diagnostics, Tape test contains tape tasks, Sensing contains raw sensor diagnostics, and Camera contains photos. Boot and entering Autonomous select Navigation; Teleop selects Motion. A saved Shuffleboard layout may retain obsolete tabs/widgets; close them or use a fresh layout after deployment.

1. Connect Control Station and keep the robot **Disabled**. Initialization is also available in enabled Autonomous and never drives motors; enabled Teleop rejects it.
2. Select the approximate **Starting area**, independently of Saved goal. This supplies default wall coordinates and a route source, never actual measured X/Y. It does not overwrite the heading field. **Current pose** uses existing odometry to determine the wall side when no named area applies.
3. Enter **Wall X mm or -** and **Wall Y mm or -**. A number is the global coordinate of the wall being observed, NOT the desired robot-center coordinate. For example X=0 is the west wall, X=2000 the east wall, Y=0 the south wall, Y=2000 the north wall; Y=1200 can reference the horizontal wall from its south side. '-' preserves that robot-pose axis.
4. Enter **Start heading deg or -**. A numeric value sets arena heading on the next normal control update using an offset against the latest gyro sample; this does not physically rotate the robot. '-' keeps the existing gyro-derived arena heading and its offset. Sampling period, IMU configuration and control-loop clock are unchanged; no gyro-zero operation or blocking wait is introduced.
5. Press **Initialize pose**. Each requested wall axis independently selects a valid, fresh, in-range sensor whose rotated beam faces that wall. The sensor offset is rotated using the requested/current heading. The strongest normal component is used; a sensor cannot supply both independent axes. The starting area is only an approximate wall-side hint. Ensure the selected beam sees the actual wall, with no storage/delivery object obstructing it.
6. Watch the combined status and current X/Y/Heading. Numeric heading and available axes apply immediately in that update even if another requested axis is waiting for a valid return. Invalid/stale readings never reset an axis. The robot remains stopped until requested wall references complete. Press Go only after Referenced is true and Autonomous is enabled.

| Wall X | Wall Y | Heading | Initialization result |
| --- | --- | --- | --- |
| Number | - | - | Correct X from that wall; preserve Y and gyro frame |
| - | Number | - | Correct Y from that wall; preserve X and gyro frame |
| - | - | Number | Set heading only; preserve X/Y |
| Number | Number | Number or - | Reference both axes using entered/current heading |
| - | - | - | Leave all pose components unchanged |

For **Delivery left physically facing -90 degrees**, use X=0, Y=0 and heading=-90 to reference the west wall with the **rear sonar** and south wall with the **right IR**. With rear range 127 mm and right IR range 340 mm, the center becomes X=127+173=300 mm and Y=340+60=400 mm. For Y-only referencing, use X='-', Y=0; X is retained exactly. For X-only, use X=0, Y='-'; Y is retained exactly. Heading '-' does not copy raw gyro yaw into the arena frame or re-zero its offset.

At heading 0 (north), west X=0 selects left IR and south Y=0 selects rear sonar. At heading +90 (west), east X=2000 selects rear sonar and north Y=2000 selects right IR. Changing the heading therefore changes the selected sensors automatically, unlike the removed fixed profiles. `navigation_config::REFERENCE_MIN_COMPONENT` controls the minimum wall-normal beam component (default 0.70).

The pose math for a selected axis is `center = wall_coordinate - rotated_sensor_offset - range * beam_component`. '-' means retain current odometry on that axis, even when physically relocating the robot; it does not infer a missing coordinate from the area's nominal pose. Reference both axes when both coordinates need to be established.

A completed reference survives enabling Autonomous. Returning to Start is ordinary navigation and does not reset pose. Changing Starting area while idle clears the reference and loads that area's default wall coordinates; pressing Initialize again is required. Selecting destinations does not require repeated initialization after arrival. To reference another physical orientation, change the heading field explicitly before Initialize.

### Route and controls

On **Navigation**, choose **Saved goal** or **Custom** and edit final X/Y/heading. Named goals use the configured default guides when **Use edited route** is off. To override, enable that toggle and enter zero or more **Edited guides** as `x,y,heading; x,y,heading`; only an explicitly edited empty list requests direct travel. Custom destinations always use the edited list. Old `x,y` entries inherit the preceding heading. Up to 32 intermediate poses are supported. All distances are millimeters; headings are degrees.

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

The named starting area remains the route source across Ready, Stop, Disable and enable. At a successful named arrival, the source becomes the destination captured when Go was pressed, even if the dashboard selection changed during travel. Manual/custom cases use the nearest named position within 75 mm only when no named source is known. **From area** shows this source. Unknown sources do not silently generate direct named transfers: select and initialize a starting area or explicitly use an edited route.

At Go, default guides are built directly from `RobotConfig.h`, rather than trusting a possibly empty text widget. **Start to Storage right** therefore captures `1500,900,-90` before the final `1700,1700,90` pose. The guide field shows the route; enable Use edited route to override it. The former Ready-only source selection could fall back to Current pose after a state change and produce an empty/direct route; named source identity no longer depends on that state or the 75 mm fallback.

Final X/Y/heading edits belong to the selected destination. Edited-guide drafts belong to the current source–destination pair. Both survive switching selections until the program restarts; durable defaults are in `navigation_config::GOALS` and `ROUTES` in `RobotConfig.h`. The override toggle resets off when the source/destination pair changes: enable it again to use that pair's draft. Changing a selection or editing a field during travel does not change the active captured route.

Press **Go to goal** to snapshot the selected route. Intermediates use a 25 mm passing radius/cross-track corridor and relaxed 10 degree heading guidance. A 100 mm lookahead, up to 25 mm corner blending and a continuous acceleration ramp smooth transitions. An explicitly requested in-place turn still rotates there. On the final leg, the goal heading is commanded while XY travel continues. There is no separate final-heading state. Completion still requires **10 mm / 1 degree** and a stationary settle.

Every control update recalculates the remaining segment, lookahead direction, distance and velocity request from the latest sensor-assisted driving pose. A valid range discrepancy can therefore change the robot's path to the next fixed waypoint without stopping, resetting the route or changing the final goal. The final leg uses the same continuous vector/ramp update rather than timed XY correction pulses.

During transit, heading correction starts above **12 degrees** of error and releases near **6 degrees**. The yaw cap is **90 deg/s throughout Navigation** (the normal Teleop cap), replacing the inherited 20 deg/s tape-test limit that delayed large turns. Tape control retains its own limit. The heading gain stays **0.8 throughout Navigation**, including the goal: the former 1.2 approach gain and separate 2.4-gain final-turn handoff are removed. On the final leg the target is the actual goal heading, so a large requested turn starts during travel. Within the last **200 mm**, only the heading tolerance tightens to **1 degree**; it stays tight for the rest of the run, avoiding threshold chatter if sensor offsets change the remaining distance. XY and yaw remain concurrent. Sensor-based global XY correction is still committed only during the stopped final settle.

Translation retains the same **350 mm/s cap** and **300 mm/s^2 acceleration**. There is no reduced-speed goal mode, mandatory XY-stop-then-turn, or timed brake/settle/correction-pulse cycle in Navigation. A continuous stopping-distance envelope accounts for the remaining distance, acceleration and **0.18 s brake response**; physical braking to stop within 10 mm is retained. The final 0.3 s settle happens only once both XY and heading meet their tolerances. The existing 15% nonzero motor-output floor remains. The route is defined by your intermediate poses: no automatic obstacle detour or collision-veto layer remains. Place the poses so the segments, corner blends and turns have room for the robot and manipulator.

### Motion tuning in one header

Edit **`src/main/include/RobotConfig.h`** for all motion tuning. The index at its top identifies each section. Angles intended for tuning are specified in degrees; their radian constants are derived automatically.

| Section | Main settings |
| --- | --- |
| `navigation_config` | Translation speed/acceleration; 25 mm intermediate and 10 mm final tolerances; lookahead/corner blend; 12/6 degree transit hysteresis; transit gain; 200 mm tolerance-tightening distance; constant heading gain; continuous final-braking response; final settle duration; range blend/step limits |
| `heading_control_config` | Final 1 degree tolerance; yaw speed/acceleration; rate damping; coasting prediction; heading recheck; stationary thresholds |
| `position_control_config` | Tape-test XY gain, speed, braking, capture radius, filtering and correction/rest/settle times (Navigation does not use these pulse phases) |
| `wheel_drive_config` | PWM floor; bounded common output scaling; neutral thresholds; encoder speed, stall and direction thresholds |
| `localization_config` | Wheel geometry/count conversion; sensor mounts; control/sample timing; Teleop speeds, acceleration and stick deadband |
| `tape_test_config` | Tape approach targets, speed/acceleration, scan/centering settings and task limits |

The tape-test XY controller reads `position_control_config`; Navigation uses its own continuous final-leg settings in `navigation_config`. Moving existing Teleop values into named constants preserves their previous defaults. No motion gain, speed limit or tolerance needs a source-file edit.

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

On Navigation, **X/Y** shows the sensor-assisted pose used for control after referencing, and the global pose during initialization. Motion's X/Y remains the global odometry pose. When readings are missing or invalid, the route continues from encoders/gyro and the last accepted driving offset; invalid values never become zero-distance corrections.

### Stopped goal update

After the normal goal pose/heading check and a 0.3-second stationary settle, only range estimates acquired during that settle and still supported by fresh valid sensor data may be committed to global XY odometry. There is no extra sensor-residual gate and no requirement that every sensor be available. If the resulting position is outside the 10 mm tolerance, the normal position controller makes the final correction. Otherwise the goal completes. Unavailable ranges are skipped, not treated as task failures.

The same valid selected ranges may establish global X/Y during explicit initialization before travel. No covariance machinery, motor trims or empirical odometry scales are added.

### Remaining stops and validation

The retained navigation protections are explicit Stop/Disable/disconnect, unavailable or invalid gyro/odometry/control data, and the existing hardware motor/encoder fault checks. Malformed/nonfinite settings are rejected before starting. There is no navigation task deadline, progress watchdog, scene-feasibility error, recovery failure or sensor-required travel gate.

The bounded late-sample handling remains: autonomous odometry preserves encoder travel through intervals up to 500 ms, while controller ramps use at most a 40 ms step. Larger/unusable feedback gaps remain control-data faults during travel. Sensor initialization can recover an old odometry health fault by resetting it, while retaining IMU/Control Station/clock checks. Old drive faults do not prevent a sensor-only reference; they remain checked before driving. Teleop driving behavior, camera capture, project 7 and build/vendor configuration are unchanged. Motion displays **Gyro raw / offset deg**, without task controls. The first value is the CCW-signed AHRS yaw; the second is the actual stored arena-heading offset, not an angle calculated from the target. Goal changes and range corrections never write either value. Only explicit numeric-heading initialization/mode synchronization establishes an offset; the goal controller reads heading. This makes physical turning, a changed reference, and sensor drift distinguishable. A raw-yaw change while physically stationary still requires a hardware observation; no synthetic heading freeze, gyro re-zero or calibration is added to hide it.

`Navigation.cpp` now contains route following, reference initialization and range support. `Autonomous.cpp` retains the shared heading/position and tape controllers. `LocalNavigation.cpp` and its planner/map classes are deleted. Existing component-check source covers beam/mount projections, invalid readings, stopped correction, control transitions and timing; motion-model simulation code has been removed. No simulations are run after the user's prohibition. Validation for this revision is compilation/build checks; physical behavior must be evaluated on the robot.
