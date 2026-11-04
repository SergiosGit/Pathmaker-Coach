# Benchmark plan: PathMaker vs Pedro Pathing

**For:** the coach running the tests, and students helping record them.
**Goal:** measure, on the bay-b bot, how PathMaker and Pedro Pathing 3 compare on speed, accuracy, repeatability, robustness and effort, with everything else held the same.
**Background:** [PATHING-PEDRO-VS-PATHMAKER.md](PATHING-PEDRO-VS-PATHMAKER.md) explains both libraries and the expected results.

## 1. What we want to learn

| Question | Answered by |
|---|---|
| Which drives a given route faster? | A1–A5, B1 |
| Which ends closer to the target, and more consistently? | A1–A5 |
| Which stays closer to the intended line on the way? | A3, A5 |
| How much does battery charge change each one? | B2 |
| How does each handle a push, or a bump on the way? | B3, B4 |
| Does driving disturb the shooter's speed control? | B5 |
| How long does setup and tuning take, and how hard is it to write a route? | F1–F4 |
| Does PathMaker 2 meet the "lower friction" criteria? | All, scored in Section 9 |

## 2. Rules that make the comparison fair

1. **Same robot, same configuration.** Same wheels, weight and odometry pods. Record the robot's weight on the test sheet.
2. **Same localizer.** Both engines read the pose from the OctoQuad's built-in localizer through Pedro's `OctoQuadLocalizer`. PathMaker runs as a Pedro `Algorithm` adapter (see the companion document, Section 4.1). Only the control law differs.
3. **Same loop.** Same OpMode structure, the same telemetry (sent to Panels every 4th loop at most), and the shooter running or off identically. Log the loop time on every loop.
4. **Same battery window.** Start each block at 13.0–13.4 V at rest. Swap batteries below 12.6 V, except in test B2. Log the battery voltage on every loop.
5. **Same floor.** Same tiles, cleaned, and the same start mark (tape cross plus heading line).
6. **Alternate the engines.** Run in blocks ABBA ABBA (A = Pedro, B = PathMaker), so battery drain, carpet wear and motor heating affect both equally.
7. **Tune each one by its own documented method, and record the time spent.** Pedro: AutoTune (Mecanum, OctoQuad, Foresight). PathMaker: its tuning OpMode (PathMaker 2), or Panels editing for PathMaker 1, **limited to 60 minutes**. Freeze both configurations and commit them before test A1. No tuning during the A and B tests.
8. **Two speed settings for each engine.** "Max": no speed limit for Pedro, `powerScaling = 1` for PathMaker. "Matched": Pedro `maxPathSpeed = 0.6` and PathMaker `powerScaling = 0.6`. Comparing at matched speed separates accuracy differences from speed differences.
9. **Ten runs per engine, test and speed setting** (N = 10) unless stated otherwise. Mark each run valid or invalid with a reason (robot hit something, wrong start, …); invalid runs are rerun, never deleted.

## 3. Measuring the truth

The localizer can't be used to judge itself, because both engines use it. Measure from outside the robot:

| What | How | Accuracy |
|---|---|---|
| End position | Mark the robot's centre with a target sticker. Measure from the tape grid with a tape measure, or a ruler square on the tiles | About ±0.25 in |
| End heading | Laser line or straightedge on a robot side, angle measured from a tile seam with a protractor or phone angle app | About ±0.5° |
| Path on the way, and timing | Phone on a tripod or ceiling mount straight overhead at 60–240 fps, with two coloured markers on the robot. Track them with *Tracker* (free, physlets.org/tracker) or a short OpenCV script | About ±0.5 in, ±10 ms |
| Start/finish time | The engine's own completion flag (logged), checked against the video | ±1 loop |

Before all other tests, run **L0** to measure localizer error. The end-position results then separate "the engine stopped in the wrong place" from "the localizer was wrong about where it was".

## 4. Tools to build first

| Tool | What it does | Size |
|---|---|---|
| `bench/Route.java` | One route definition used by both engines: target poses, heading mode, end behavior | ~100 lines |
| `bench/PathMakerAlgorithm.java` | PathMaker's control law behind Pedro's `Algorithm` interface | ~150 lines |
| `bench/BenchOpMode.java` | Panels choices at init: engine, test, speed setting, run number. Runs one test and writes one CSV | ~200 lines |
| `bench/BenchLogger.java` | Writes one CSV row per loop to `/sdcard/FIRST/bench/<test>_<engine>_<speed>_<run>.csv` | ~80 lines |
| `docs/bench/analyze.py` | Reads all CSVs and the hand-measured sheet, computes the metrics in Section 6, and draws the plots | ~200 lines |
| Test sheet | Spreadsheet: run ID, engine, test, battery at start, measured end x / y / heading, valid, notes | — |

CSV columns, one row per loop:

```
t_s, engine, test, run, step, target_x, target_y, target_h_deg,
pose_x, pose_y, pose_h_deg, vel_x, vel_y, vel_h_degs,
power_fwd, power_str, power_turn, battery_v, loop_ms,
busy, at_parametric_end, shooter_target_rpm, shooter_rpm
```

Coordinates in the logs are Pedro's (inches, field-corner origin, counterclockwise heading), whichever engine is running. The conversion is in the companion document, Section 4.2.

Start pose for all tests: **(72, 24, 90°)** in Pedro coordinates. That is the middle of the field width, one tile from the near wall, facing up the field, leaving 96 in of clear floor ahead and 72 in either side. Adjust it to the practice field's actual clear space, and keep it the same for every run.

## 5. Tests

### 5.1 Localizer and loop checks (once, before the comparison)

| ID | Test | Procedure | Pass |
|---|---|---|---|
| **L0** | Localizer accuracy | Push the robot by hand 96 in forward along a tape line and back, 5 times. Then turn it 10 full turns and back to the start line. Compare the logged pose with the measured truth | Error < 1 in per 96 in, < 1° per 10 turns. Otherwise redo the OctoQuad tuning |
| **L0s** | Localizer with the shooter running | Repeat L0 once with the shooter spinning up and down, to check the shooter's OctoQuad setup doesn't disturb the localizer (companion document, Section 1.3) | Same as L0, and no pose jumps when the shooter starts |
| **L1** | Loop time | Hold position for 30 s with each engine, with the shooter at 4500 RPM | Report mean, 95th percentile and max loop time. Target: 95th percentile < 25 ms |

### 5.2 Path tests (A): N = 10 per engine and speed setting

Pedro coordinates, start (72, 24, 90°):

| ID | Route | Heading | End | What it shows |
|---|---|---|---|---|
| **A1** | Straight forward 48 in to (72, 72) | Constant 90° | Stop | Basic speed and stopping |
| **A2** | Strafe right 48 in to (120, 24) | Constant 90° | Stop | Sideways moves (mecanum strafe is slower and slips more) |
| **A3** | Diagonal to (108, 60), turning to 0° | Linear 90° → 0° | Stop | Moving and turning together; straightness of the line |
| **A4** | Turn in place 180°, to (72, 24, 270°) | — | Stop | Heading control alone |
| **A5** | Three steps: (72, 72) → (108, 96) → (72, 96), passing through the first two without stopping, heading constant 90° | Constant | Pass, pass, stop | Rolling transitions: PathMaker with negative `rampReach`; Pedro with chained `Paths.path(...)` |
| **A6** | Curve: quarter turn from (72, 24) heading up to (108, 60) heading right, close to a circle of radius 36 in centred at (108, 24) | Tangent (90° → 0°) | Stop | Curves. Pedro: `Paths.curve(start, (72, 60), end)`, a Bézier curve whose start and end directions match the robot. PathMaker: 4 pass-through waypoints on the arc. Optional for PathMaker 1 |

### 5.3 Robustness tests (B)

| ID | Test | Procedure | N |
|---|---|---|---|
| **B1** | Game-like cycle | Start → shooting spot → pickup spot → shooting spot, three times. Pause 1.5 s at the shooting spot with the shooter at the target RPM and the intake feeding. Choose the spots from the BIOBUZZ field layout and use the same ones for both engines | 5 per engine |
| **B2** | Battery effect | A1 and A3 at **13.3 V** and again at **12.3 V** (run the battery down on a load, not by driving tests) | 5 per engine per voltage |
| **B3** | Push while holding | After A1 finishes, push the robot 6 in sideways with a spring scale at about 30 N, then release | 5 per engine |
| **B4** | Bump on the way | During A1, a foam pool noodle fixed at 24 in brushes the robot's side | 5 per engine |
| **B5** | Shooter while driving | Run B1 with the shooter at 4500 RPM. Compare the shooter's speed error during driving with its error standing still | from B1 |

For B3, PathMaker 1 has no hold mode. Record "no correction" as the result; that is a finding in itself.

### 5.4 Effort tests (F)

Each is done by a student who hasn't used that engine on this robot before. Use one student per engine, or the same student with the order swapped, and time it with a stopwatch.

| ID | Task | Measure |
|---|---|---|
| **F1** | Setup: from a fresh checkout to the robot driving A1 for the first time | Minutes; number of help requests |
| **F2** | Tuning, until A1 ends within 1 in and 2°, three times in a row | Minutes; number of runs |
| **F3** | Recovery: the drive wheels are swapped for worn ones (or 1 kg is added). Retune until F2 passes again | Minutes |
| **F4** | Write a new three-step route (given on paper) and run it | Minutes; number of mistakes before it runs correctly |

Also count the lines of route code for A5 and B1 in each engine, and the number of constants that had to be set.

## 6. Metrics

All computed by `analyze.py`. Report each as mean, standard deviation and worst case over the valid runs.

| Metric | Definition |
|---|---|
| **Time to done** | Start command to the engine's "done" flag (`!isBusy()` for Pedro, target zone for PathMaker) |
| **Time to settle** | Start command to the first moment the externally measured (video) pose stays within 0.5 in and 1° of the target, and speed stays < 1 in/s, for 0.25 s |
| **End error, measured** | Distance in x/y and heading difference between the tape-measured end pose and the target |
| **End error, logged** | The same from the localizer. Measured minus logged shows the localizer's share |
| **Repeatability** | Standard deviation of the measured end positions (spread of the 10 dots) |
| **Off-line error** | For A1–A3 and A5: the largest and RMS perpendicular distance between the robot (from video) and the straight line between start and target. For A6: distance from the Bézier curve that Pedro was given |
| **Overshoot** | Largest distance past the target along the direction of travel |
| **Power used** | ∫\|wheel power\| dt, as a proxy for battery use |
| **Battery sensitivity** (B2) | Change in time to done and end error between 13.3 V and 12.3 V |
| **Hold stiffness** (B3) | Peak displacement and time to return within 0.5 in |
| **Bump recovery** (B4) | Largest off-line error after the bump, and extra time to done |
| **Shooter disturbance** (B5) | 95th-percentile shooter RPM error while driving, minus the same while standing still |
| **Loop time** | Mean, 95th percentile and max |
| **Effort** (F) | Minutes and counts as listed in 5.4 |

## 7. Procedure for each run

1. Check the battery voltage, and swap it if needed (rule 4).
2. Place the robot on the start mark: centre on the cross, side along the heading line.
3. In Panels during init, set the engine, test, speed setting and run number. Check the logged pose reads (72, 24, 90°) ± 0.1.
4. Start the video, then press start on the Driver Hub.
5. When the robot is done and still, stop the OpMode, then stop the video.
6. Measure the end pose and write it on the test sheet with the run ID.
7. Note anything unusual (wheel slip, a sound, a hit).

A block of 10 runs takes about 15 minutes. The full A matrix (6 tests × 2 engines × 2 speed settings × 10 runs = 240 runs) takes about 6 hours of field time. Spread it over three sessions, each one starting with L0 (5 minutes) to catch localizer drift.

## 8. Analysis

1. Discard nothing. List invalid runs and their reasons in the report.
2. For each test and speed setting, plot:
   - the end positions of the 10 runs per engine as dots around the target (a "target plot");
   - the paths from video, overlaid;
   - box plots of time to done.
3. **Differences:** for time and end error, compare the engines with a Welch t-test (unequal spread allowed) and report the difference with a 95% confidence interval. With N = 10, a difference smaller than about one standard deviation is not reliable. Say "no clear difference" rather than picking a winner.
4. Add the "measured minus logged" error from L0 to explain end errors. If both engines show the same offset, it's the localizer, not the engine.
5. Write up in `docs/PATHING-BENCHMARK-RESULTS.md` with the plots, the scorecard and the raw data location.

## 9. Scorecard

Score each category from 1 to 5 against the other engine, using the results above.

| Category | Weight | Measured by |
|---|---|---|
| Speed | 20% | A1–A5, B1 time to done (max setting) |
| Accuracy | 20% | A1–A5 measured end error (matched setting) |
| Repeatability | 10% | Standard deviation of end positions |
| Path shape | 10% | Off-line error A3, A5, A6 |
| Robustness | 10% | B2–B4 |
| Coexisting with mechanisms | 5% | L1, B5 |
| Setup and tuning effort | 15% | F1–F3 |
| Writing routes | 10% | F4, lines of code, constants count |

**PathMaker 2 counts as a lower-friction alternative** if **all** of these hold:

| Criterion | Threshold |
|---|---|
| Setup + tuning (F1 + F2) | ≤ 50% of Pedro's minutes, and ≤ 30 minutes |
| Speed | Time to done ≤ 125% of Pedro's on A1–A3 and B1 (i.e. ≥ 80% of Pedro's speed) |
| Accuracy | Mean measured end error within 1 in and 3° of Pedro's, on A1–A5 at the matched setting |
| Route writing (F4) | ≤ 10 minutes, with no more mistakes than with Pedro |
| Loop time | 95th percentile ≤ Pedro's + 5 ms |

If it misses only on speed or curves, it may still be the right teaching tool. The report should say which kind of robot and team each engine suits.

## 10. Schedule

| Week | Work |
|---|---|
| 1 | SDK 12.0 upgrade and merge (companion document, Section 1.2). Pedro AutoTune on the bay-b bot. L0 and L1 |
| 2 | `Route`, `PathMakerAlgorithm`, `BenchOpMode`, `BenchLogger`. Dry run of each test with each engine (1 run each) |
| 3 | Effort tests F1–F2 with fresh students. Freeze both configurations |
| 4–5 | A matrix (3 sessions) |
| 6 | B tests, F3–F4, analysis, results document |
| After phase 3 of the PathMaker 2 roadmap | Repeat A1–A3, B1, F2 and F4 with PathMaker 2, and update the scorecard |

## 11. Risks to the plan

| Risk | Effect | What to do |
|---|---|---|
| Pedro 3 point releases during testing | Results from different versions mixed together | Pin `revhub` and `tuning` versions for the whole run. Record them in every CSV header |
| Not enough clear floor | A2, A3 and A6 can't run at full length | Scale all routes by the same factor and note it. Never shorten only one engine's runs |
| Overhead video is hard to set up | No path-shape data | End-point measurements still answer most questions. Use the logged pose for path shape and say so |
| Shooter's OctoQuad setup resets the localizer (L0s fails) | Pose jumps; invalid results | Fix the startup order first (companion document, Section 1.3) |
| Effort tests depend on one student | Results reflect that student | Use two students per engine if possible, or at least swap the order |
