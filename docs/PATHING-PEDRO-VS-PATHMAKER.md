# PathMaker and Pedro Pathing on the bay-b bot

**For:** the coach, and students who will maintain the drive code.
**Covers:** a review of the plan to merge the bay-b bot code into our TeamCode, a technical comparison of PathMaker and Pedro Pathing 3, and a roadmap for making PathMaker a lower-friction alternative.
**Companion:** [PATHING-BENCHMARK-PLAN.md](PATHING-BENCHMARK-PLAN.md), which tests the claims made here on the robot.

Sources: our TeamCode, the bay-b bot code (then in `BabyBot/teamcode`, now imported into TeamCode's `pedro/` and `babybot/` packages), and Pedro's library source, Quickstart and documentation as of 3 Nov 2026 (`com.pedropathing:revhub:3.0.2`, `com.pedropathing:tuning:1.0.3`, `com.pedropathing.ivy:pedro:1.1.1`). Nothing in this document has been built or run.

## The short version

- **The plan is sound, with one change of order.** Before merging, move to **FTC SDK 12.0**, since Pedro 3 is built against it. That means redoing PR #6, which upgrades to 11.2.1. SDK 12.0 also breaks our AprilTag code at compile time.
- **The bay-b bot code has never followed a path.** `Constants.create()` returns `null` (`BabyBot/teamcode/pedro/Constants.java:11`), so every OpMode that uses the follower would crash. Only the motor-direction and OctoQuad tuners can have run. AutoTune's results only appear on its web page, so if nobody pasted them into `Constants.java`, they are gone and must be re-measured.
- **The OctoQuad is shared**, so plan for it. Pedro uses two OctoQuad channels for odometry pods (ports 0 and 1 by default). The shooter's Through Bore was on channel 0 and has been moved to channel 2 (`ShooterConfig.encoderChannel`), and the shooter also reconfigures the OctoQuad's channel bank at startup. Still test whether the localizer survives the shooter starting up.
- **Pedro 3 and PathMaker solve different problems.** Pedro follows curved paths with a braking model it measures itself. PathMaker drives each axis (x, y, heading) toward a goal with hand-set ramps and per-axis delays. Pedro will very likely be faster and more accurate. PathMaker's strengths are a smaller learning curve, no outside dependency, live Panels tuning and a simulator.
- **The one fair comparison holds everything else fixed:** the same localizer (the OctoQuad's built-in one), the same drive output, the same loop and the same logging. The cleanest way is to run PathMaker's control law inside Pedro's `Follower`, by writing a small adapter for Pedro's `Algorithm` interface. The PathMaker library itself keeps no Pedro dependency.
- **"Fully on Panels" works for everything except Pedro's tuning.** Pedro 3's AutoTune is its own web page on the robot (http://192.168.43.1:10158), separate from Panels (port 8001). Telemetry, graphs and live configuration can all live in Panels.
- **To make PathMaker lower-friction,** borrow five ideas from Pedro without the complexity:
  - a non-blocking `update()`
  - a pluggable localizer and drivetrain
  - paths as data, with alliance mirroring
  - braking from a measured stopping distance instead of hand-set ramp lengths
  - a short self-measuring tuning OpMode

  Section 5 has the roadmap.

---

## 1. Review of the plan

The plan has five parts:
1. Merge the bay-b bot code into our TeamCode.
2. Add the shooter PID.
3. Use Panels everywhere.
4. Be able to run either PathMaker or Pedro.
5. Benchmark the two, and improve PathMaker using Pedro's practices.

All five fit together. The points below decide whether the merge goes smoothly.

### 1.1 What the bay-b bot code contains

| File | What it is | State |
|---|---|---|
| `pedro/Constants.java` | Should build the `Follower` | **Returns `null`.** No drivetrain, localizer or Foresight configuration exists |
| `pedro/Tuning.java` | AutoTune procedure list | Mecanum and OctoQuad tuners enabled. Foresight and Tests commented out, because they need the missing configurations |
| `pedro/procedures/*` | Pedro Quickstart tuners, copied unchanged | Fine. In `Tests.java:171,213,255` the line, curve and interpolation tests ignore the distance entered in AutoTune (a local `distance = 48` hides it) |
| `hw/Robot.java`, `hw/Drivetrain.java`, `op/BaseOpMode.java`, `math/ControlManager.java` | An Ivy-based TeleOp skeleton: field-centric drive with "hold position when sticks are idle" | Would crash, because the follower is `null`. `Drivetrain` also grabs the four drive motors and sets their directions itself, which competes with Pedro's `MecanumConfig` |
| `op/test/DriveTest.java` | TeleOp using the skeleton | Crashes for the same reason |
| `op/test/LocalizingClump.java` | Limelight ball finder that drives to the ball it sees | Crashes (null follower). It also has a heading bug: lines 87–93 pass radians from `follower.pose().heading()` into `PoseFactory.degrees()`, which converts them as degrees, so the heading collapses to almost zero. The ball offset, measured relative to the robot, is also added to field coordinates without rotating it by the robot's heading |
| `op/test/TipDetector.java` | Detects tipping when the webcam loses sight of AprilTags | Independent of pathing. Uses only the detection count, so it should still compile on SDK 12.0 |

There's no Gradle file in the copy, so the Pedro, Ivy and Panels versions are taken from the imports and from Pedro's current Quickstart.

### 1.2 Order of work

1. **Upgrade to FTC SDK 12.0.0** (the BIOBUZZ season release).
   - Pedro `revhub` 3.0.2 is compiled against 12.0.0, and the Quickstart uses it.
   - 12.0 also adds OctoQuad firmware v3.1 support.
   - It breaks code that reads `AprilTagDetection` fields directly. We do that in `hw/VisionSystem.java` and `pathmaker/PathMakerStateMachine.java:49`. Detections must now be cast to `AprilTagSingleDetection` or `AprilTagClusterDetection`.
   - **BIOBUZZ AprilTags move**, so FIRST says they can't be used for absolute field localization. PathMaker's "rebase on AprilTag" feature (`PathManager.moveRobot`) has no use this season. Leave it switched off.
   - Practical step: redo PR #6 as `sdk-12.0`, and rebase PR #7 (shooter) onto it.
2. **Add dependencies** to `build.dependencies.gradle`:
   ```gradle
   repositories { maven { url 'https://repo.dairy.foundation/releases/' } }   // needed by the tuning library
   dependencies {
       implementation 'com.pedropathing:revhub:3.0.2'
       implementation 'com.pedropathing:tuning:1.0.3'
       implementation 'com.pedropathing.ivy:pedro:1.1.1'      // only if we keep the Ivy command style
   }
   ```
   Panels (`com.bylazar:fullpanels`) stays in `TeamCode/build.gradle`. Pedro pulls in `kotlin-stdlib` 2.3.21, which our Kotlin 2.4.10 plugin handles.
3. **Bring in `pedro/` unchanged, then fill in `Constants.java`** by running AutoTune: Mecanum, then OctoQuad, then Foresight (Section 3.4).
4. **Make the OctoQuad shareable** (Section 1.3).
5. **Add the bay-b bot to `RobotConfig.initializeRobotType`**, keyed by its Control Hub's `ANDROID_ID`, like ScootBot and SeasonRobot. Its drive motor names (`front_left` and so on) and left-side reversal already match our defaults.
6. **Make PathMaker non-blocking and give it the same pose source** (Section 5, phase 1). Before this, the two can't run side by side fairly.
7. **Add the engine switch and the benchmark OpModes** (Section 4, and the benchmark plan).
8. **Fix `RobotConfig.Camera.customCameraCalibration()`** (`configuration/RobotConfig.java:132`). It calls itself and would crash with a stack overflow the first time it runs. Nothing calls it yet.

### 1.3 Sharing the OctoQuad between odometry and the shooter

| Item | Pedro localizer | Shooter (PR #7) | What to do |
|---|---|---|---|
| Channels | Forward pod on port 0, strafe pod on port 1 (defaults) | Through Bore on `encoderChannel = 2` (moved from 0) | Done: channels 0 and 1 are left for the pods |
| Channel bank | Assumes quadrature | Sends `setChannelBankConfig(ALL_QUADRATURE)` at startup | Skip the command if `getChannelBankConfig()` already reads `ALL_QUADRATURE`. Create the shooter **before** the follower. Then run benchmark test L0 to confirm the pose isn't reset |
| Velocity window | `localizerVelocityIntervalMS` for the localizer | Per-channel window for the shooter channel (20 ms) | Separate settings; no conflict expected |
| Device type | Pedro's docs say configure as `OctoQuadFTC`, on an I2C port **other than 0** | Uses `hardwareMap.tryGet(OctoQuad.class, "octoquad")` | Configure the way Pedro says, then repeat tuning step 2 from SHOOTER-PID.md to check the shooter still reads |
| I2C time per loop | One localizer block read | One velocity read | Two transactions per loop. Measure the loop time (test L1) |

The OctoQuad's built-in localizer is a real step up for both libraries. It merges the pods with its own IMU at a fixed rate, independent of our loop. PathMaker today integrates dead-wheel ticks once per 40 ms loop (`op/RobotPose.java:225,308`).

---

## 2. How each library works

### 2.1 PathMaker (ours)

| Part | Where | How it works |
|---|---|---|
| Path definition | `PathDetails.java` (667 lines) | One `case` per path in a `switch`. Each sets static fields: a field goal for x, y and heading, a start delay for each, `powerScaling`, `pathTime_ms`, ramp lengths, and mechanism commands such as shooter RPM and intake power. Red and blue versions are separate copies |
| Sequencing | `PathMakerStateMachine.java` | Goes through `autoPathList` in order. A path ends when all three axes are inside their target zone, or `pathTime_ms` runs out |
| Control law | `PathManager.calculateCorrectionPower` | Per axis, in field coordinates. Full power outside `rampReach`, then `power = error / rampReach` (a P controller with gain 1/rampReach), with a cap on how fast power rises (`maxPowerStepUp`). An "energy" check cuts power to 0 when moving too fast near the goal. A negative `rampReach` means "drive through the goal at full power" (a rolling transition) |
| Power sharing | `PathManager.balancePower` | If \|x\|+\|y\|+\|turn\| > 1, all three are scaled so the sum is 1 |
| Localization | `RobotPose.readPose` | Three dead wheels on the hub encoder ports, or four drive encoders plus the IMU, or a simulator. Integrated once per loop; velocity is the change since the last loop |
| Loop | `PathManager.moveRobot` | **Blocking:** sleeps to make each cycle 40 ms (`PathManager.java:129`) |
| Tuning | Panels `@Configurable` statics | Everything is live-editable. No measurement procedure |

**Strengths**
- **Simple idea:** "go to this x, y, heading, and start each one after its own delay". Students can predict what the robot will do.
- **Per-axis delays and rolling stops** are a simple way to plan around obstacles and to choreograph mechanisms.
- **No outside dependency:** it survives library rewrites, such as Pedro 2 → 3.
- **Everything is visible and tunable in Panels**, and there's an offline simulator (`RobotPoseSimulation`).

**Weaknesses found in the code**

| # | Issue | Effect | Location |
|---|---|---|---|
| W1 | Blocking 40 ms sleep in the drive update | Caps the whole robot at 25 Hz. The shooter PID and other mechanisms only update between drive cycles | `PathManager.java:129` |
| W2 | x and y are controlled separately | The robot doesn't drive straight to the goal. It moves diagonally, then finishes along whichever axis is left. The path shape depends on distances and ramps | `PathManager.moveRobot` |
| W3 | Pure P control near the goal, with no friction term (`kS`) or minimum power | Power fades toward zero before the target zone, so paths often end on `pathTime_ms` instead of arriving | `PathManager.java:247` |
| W4 | "Energy" braking is all-or-nothing | Power drops to 0 above a fixed energy threshold, then P control resumes. This gives uneven stops that depend on robot mass and battery | `PathManager.java:244` |
| W5 | VELOCITY ramp mode divides by `deltaPosition`, which is always 1.0 (its assignments are commented out) | The ratio has wrong units. The mode is unreliable | `PathManager.java:67,264` |
| W6 | Power sharing by the sum of axes | A diagonal move with turning gets at most ⅓ power per axis, even when the wheels have more to give. Pedro scales by the actual largest wheel power | `PathManager.java:154` |
| W7 | Path complete without checking speed | Can end a path while still moving fast, unless the ramps happen to prevent it | `PathManager.java:136` |
| W8 | Odometry integrates once per 40 ms, without arc correction; velocity is a raw difference | Pose error grows during fast turns, and the noisy velocity feeds the energy check | `RobotPose.java:225,308` |
| W9 | No battery-voltage compensation | Paths change as the battery drains | — |
| W10 | All state is in public statics | Hard to unit test, two instances can't coexist, and things can be changed from anywhere | throughout |
| W11 | Paths written by copy-paste, separate for each alliance | 30 near-identical blocks. A fix must be repeated in each | `PathDetails.java:194–667` |

W1, W3, W5 and W7 are bugs or near-bugs that are worth fixing whatever the benchmark shows.

### 2.2 Pedro Pathing 3

Pedro 3 (released 2026) is a full rewrite of Pedro 2. Tutorials and code written for Pedro 1.x/2.x **don't apply**: the Follower constructor, configuration style, path API and tuning are all different.

| Part | How it works |
|---|---|
| Structure | `new Follower(localizer, drivetrain, algorithm)`. Each part is an interface, so the drivetrain, the localizer and the path-following law can each be swapped |
| Paths | `Paths.line(a, b)`, `Paths.curve(a, c1, …, b)` (Bézier curves), and `Paths.path(p1, p2, …)` to chain them. Heading modes: tangent, linear, constant, facing a point, piecewise, custom |
| Poses | `PoseFactory.degrees()` / `.radians()`, with `mirrorX`/`mirrorY` so one alliance's route generates the other's |
| Algorithm: Foresight | Uses the robot's measured top speed, natural deceleration and braking coefficients to decide how fast it may go while still stopping on time. Proportional controllers handle position and heading; the position one switches gain at 2.5 in of error |
| Ending a path | Complete when progress along the path is ≥ 0.975 **and** position, heading and speed are within tolerance (defaults 0.1 in, 0.4°, 0.1 in/s), or after a 100 ms timeout. Then it holds position |
| Limits | `maxPathSpeed`, maximum velocity, acceleration and deceleration, `maxBrakingPower` (0.2), `brakeAggression` |
| Localizers | OctoQuad (uses its built-in localizer), Pinpoint, OTOS, two- and three-wheel |
| Tuning | AutoTune web page on the robot (port 10158). It drives the robot through measurement runs and generates the Java configuration to paste in |
| Commands | Ivy: `follow(follower, path)`, `hold(...)`, `sequential`, `parallel`. Mechanism actions run alongside driving without blocking |
| Logging | `follower.withLogger(log -> ...)` receives position, localizer, drivetrain and algorithm data on every `update()` |
| Coordinates | Inches, origin at a field corner, heading counterclockwise-positive with 0 facing +x. Different from the FTC SDK and from PathMaker |

**Strengths**
- Fast, accurate following, with a stopping model it measures itself.
- Curved paths and heading control.
- AutoTune measures about 20 constants with no hand tuning.
- Built-in mirroring, logging and command structure.
- Large user community, a path visualizer, and support on Discord.

**Weaknesses and risks**

| # | Risk |
|---|---|
| P1 | **Very new.** revhub 3.0.2 was published 16 Oct 2026, and AutoTune 1.0.3 on 30 Oct 2026. The documentation is thin in places: the install page shows a `x.y.z` placeholder, and the Quickstart's `Constants.create()` returns `null` until you fill it in. Expect fixes in point releases |
| P2 | Tied to FTC SDK 12.0, and pulls in Kotlin stdlib, Sloth and nanohttpd. Library updates can force project changes |
| P3 | AutoTune needs open floor (runs of 36–48 in or more) and a separate web page. Configurations must be pasted in by hand. A drivetrain change (wheels, weight) means re-tuning |
| P4 | Students mostly can't see how Foresight works. When it misbehaves, debugging means reading the library source |
| P5 | Different coordinate frame from the SDK and PathMaker. Every pose shared between them needs converting (Section 4.2) |
| P6 | `Mecanum.drive` sends powers with no battery-voltage compensation. Foresight's speed model was measured at one battery level, so performance may drift with charge (benchmark test B2) |

---

## 3. Head-to-head

### 3.1 Feature comparison

| Capability | PathMaker today | Pedro 3 |
|---|---|---|
| Point-to-point move | Yes (one axis at a time) | Yes (straight line) |
| Curved paths | No; approximated with several waypoints | Yes (Bézier) |
| Heading control | Goal plus start delay | Six interpolation modes |
| Staggered axes (start strafing later) | **Yes, built in** | Use a curve or a chain of paths |
| Rolling transition between paths | Negative `rampReach`, or `pathTime_ms` | Chained paths. `brakeAtEnd = false`, or finish on `atParametricEnd()` |
| Braking | Hand-set ramp lengths and energy cut-off | Measured braking model |
| Path complete | Position zone, or timeout | Progress + position + heading + speed, or timeout |
| Hold position | Teleop heading hold only | Yes (`hold`) |
| Battery compensation | No | No |
| Loop | Blocking, 25 Hz | Non-blocking, as fast as the loop runs |
| Localizer | Its own (hub dead wheels, IMU, simulator) | Pluggable, including the OctoQuad's built-in one |
| Alliance mirroring | Copy-paste | `PoseFactory.mirrorX` |
| Running mechanisms alongside | Fields on each path step | Ivy commands, or the user's state machine |
| Tuning | Edit in Panels by trial and error | AutoTune (about 15–30 min of runs) |
| Live dashboard | Panels | Panels for telemetry; AutoTune page for tuning |
| Offline simulation | Yes | No |
| Outside dependency | None | Pedro, Ivy, Sloth, Kotlin stdlib |
| Code a student reads to understand it | About 1,000 lines | Library internals, plus a ~200-line Constants file |

### 3.2 Expected results (to be checked by the benchmark)

| Measure | Expectation | Why |
|---|---|---|
| Time for a 48 in move | Pedro faster | Uses full available speed and a measured braking point; PathMaker uses power scaling and conservative ramps |
| End position accuracy | Pedro better, if both use the same localizer | Pedro's end check includes speed and heading |
| Curved or multi-step routes | Pedro clearly better | PathMaker has no curves |
| Setup time, from nothing to a first path | PathMaker shorter, if odometry already works | No tuning procedure |
| Time for a student to change a route | Close | Both are now "edit poses in a file" |
| Robustness to battery charge | Both drift | Neither compensates |

### 3.3 Where each fits

- **Pedro:** competition autonomous with several cycles and curved routes, when the team has an hour of open field for tuning and someone willing to follow Pedro updates.
- **PathMaker:** teaching control (every term is visible and changeable), robots without good odometry, quick test routines, and anyone who wants to own all of their code.

### 3.4 AutoTune order for the bay-b bot

1. **Mecanum** procedure: motor names and directions → `MecanumConfig`.
2. **OctoQuad** procedure: port, pod type (goBILDA four-bar = 505.3 ticks/in, swing-arm = 336.9), heading scalar (10 turns), pod directions and offsets → `OctoQuadConfig`.
3. Enable the Foresight tuner and Tests in `Tuning.java`, pointing them at `OctoQuadLocalizer`. As written, the commented-out `Tests` line uses `PinpointLocalizer`, so change it.
4. **Foresight** procedure (about 10 runs; needs at least 48 in forward and sideways) → `ForesightConfig`.
5. Paste all three configurations into `Constants.java`, and make `create()` return `new Follower(new OctoQuadLocalizer(h, localizerConfig), new Mecanum(h, drivetrainConfig), new Foresight(foresightConfig))`.
6. Run the Hold, Line and Curve tests from AutoTune.
7. **Commit `Constants.java` right away.** The tuning results exist only on the AutoTune page until then.

---

## 4. Running either library on one robot

### 4.1 Architecture

```
                      ┌──────────────── one OpMode loop (non-blocking) ───────────────┐
 Route (data) ──────► │  PathingEngine  ── PEDRO ──► Follower(Foresight)              │
 poses, heading mode, │                  └ PATHMAKER ► Follower(PathMakerAlgorithm)   │
 end behavior, actions│        both use: OctoQuadLocalizer + Mecanum drivetrain       │
                      │  Shooter.update()   Intake …   BenchLogger   Panels telemetry │
                      └────────────────────────────────────────────────────────────────┘
```

- **Route:** one definition of the route, used by both engines: a list of steps, each with a target pose (inches, degrees), a heading mode, end behavior (stop, hold, or pass through) and mechanism actions. The Pedro executor turns each step into `Paths.line(...)` with `.linear(...)` or `.constant(...)`. The PathMaker executor turns it into goals and delays.
- **PathMakerAlgorithm:** a small class (about 150 lines) that implements Pedro's `Algorithm` interface. It computes PathMaker's per-axis powers from Pedro's `MotionState` and returns `DrivePowers` (forward, strafe, turn, all relative to the robot). For the benchmark, this is the fairest arrangement: same localizer, same motor output, same loop and same logging, so only the control law differs. It lives in a `bench` package. The PathMaker library itself needs no Pedro classes.
- **Engine switch:** a `@Configurable` enum `PathingConfig.engine` that can be changed in Panels during init. Also add one OpMode per engine with a fixed engine (e.g. `Auto Pedro: Route A`, `Auto PathMaker: Route A`), so drivers can't pick the wrong one by accident in a match.
- **Telemetry:** use one pattern everywhere. Either `JoinedTelemetry(telemetry, PanelsTelemetry...getFtcTelemetry())`, as in our OpModes, or the bay-b bot's `TelemetryManager`. Graph values with the same names for both engines (`pose_x`, `err_xy`, `loop_ms`, …) so the Panels graphs line up.

### 4.2 Coordinates

| | PathMaker | Pedro |
|---|---|---|
| Origin | Robot start (or the offsets in `PathDetails`) | Field corner, (0, 0) to (144, 144) in |
| Axes | +y = robot forward at start, +x = right | +x = right, +y = up the field |
| Heading | Degrees, clockwise positive, 0 = facing +y | Radians, counterclockwise positive, 0 = facing +x |

For a robot that starts at Pedro pose (X₀, Y₀) facing up the field (90°), a PathMaker goal (x, y, a) is the Pedro pose `(X₀ + x, Y₀ + y, 90° − a)`. Put this conversion in one helper method that both executors use, and test it before any driving.

### 4.3 Moving fully to Panels

| Need | Where |
|---|---|
| Live tuning of PathMaker, shooter and route parameters | Panels Configurables |
| Graphs (errors, powers, RPM, loop time) | Panels Graph |
| Field view of the pose | Panels field plugin, drawing from `follower.pose()` for both engines |
| Pedro tuning | AutoTune page (port 10158). Panels can't replace it |
| Benchmark logs | CSV on the Control Hub (`/sdcard/FIRST/bench/`), downloaded afterwards. Panels is for watching live, not for keeping data |

---

## 5. Roadmap: PathMaker 2, a lower-friction alternative

The goal: keep PathMaker's simple model of "go here, start each axis when told", while removing the hand tuning and the bugs. Each phase is useful on its own. The benchmark is run after phase 1, and again after phase 3.

| Phase | Change | Borrowed from Pedro | Fixes |
|---|---|---|---|
| **0. Bugs** | Remove the VELOCITY mode, or fix `deltaPosition`. Add a minimum power (`kS`) near the goal. Add a speed condition to path completion | End constraints | W3, W5, W7 |
| **1. Plumbing** | Non-blocking `update(dt)`: no `Thread.sleep`. Small interfaces `PoseSource`, `DriveOutput`, `Clock`. Adapters for the OctoQuad localizer (Pedro's `OctoQuadLocalizer`, or the SDK's localizer calls directly), our dead wheels and the simulator. Move statics into a `PathMaker` object | Follower/Localizer/Drivetrain split | W1, W8, W10 |
| **2. Paths as data** | `Route.builder().to(x, y, a).delayX(500).passThrough().action(shooterOn).build()`. One alliance's route generates the other's by mirroring. The `switch` becomes data | PoseFactory, Paths | W11 |
| **3. Physics instead of guesses** | Drive the translation as **one 2D vector** toward the goal (keep per-axis mode as an option). Start braking when the remaining distance ≤ measured stopping distance (`v²/(2·deceleration)` × margin). Use feedforward + P on velocity (`kS + kV·v_target + kP·(v_target − v)`), position P only at the end. Scale by the largest wheel power, not the sum of axes. Battery-voltage compensation | Foresight's measured deceleration and braking, `maxScaling` | W2, W4, W6, W9 |
| **4. Self-measuring tuning** | One OpMode, three runs per axis: full power for 1 s, then let go. It measures top speed, natural deceleration and `kS`, and shows the values in Panels to copy into the configuration. About 5 minutes, about 2 m of floor | AutoTune, much reduced | removes ramp tuning |
| **5. Tests** | Run the control law against the simulator on a laptop (like `docs/shooter_sim`). Each route checked offline before going on the robot | — | regressions |

**What to keep:** per-axis delays, "pass through the goal" transitions, `@Configurable` everything, the simulator, and a library small enough to read in an afternoon (target under 600 lines).

**What not to copy:** Bézier curves (waypoints with pass-through cover most FTC routes), the command framework (our state machine is fine), and Pedro's large configuration surface.

**When PathMaker counts as lower friction:**
1. A first route runs after **under 30 minutes** of setup and tuning on a robot whose odometry already works. The benchmark measures this as F1–F3.
2. It achieves **at least 80% of Pedro's speed** on test routes A and B.
3. It achieves **end accuracy within 1 in and 3°** of Pedro's.
4. A student can **write a new three-step route in under 10 minutes**.

---

## 6. Decisions for the coach

| Decision | Recommendation | Why |
|---|---|---|
| SDK version for the merged repo | 12.0.0 | Required by Pedro 3, and this season's AprilTag library |
| Keep the Ivy command style from the bay-b bot? | Optional. Use it in Pedro-only OpModes; benchmark OpModes use a plain loop | Keeps the comparison simple; PathMaker doesn't need Ivy |
| PathMaker localizer on the bay-b bot | The OctoQuad's built-in localizer, the same as Pedro's | Fair comparison, and better than integrating once per 40 ms |
| PathMaker in the benchmark | As a Pedro `Algorithm` adapter | Isolates the control law |
| Shooter encoder channel | Channel 2 or higher | Ports 0 and 1 are the odometry pods |
| Where tuned constants live | `pedro/Constants.java` and `configuration/*Config.java`, committed after every tuning session | AutoTune results otherwise exist only in the browser |
