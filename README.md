# PathMaker Coach

A coach's FTC robot software workspace for the 2026–27 **BIOBUZZ** season. It holds:

- **PathMaker:** a small autonomous pathing library written for students, designed to need little tuning.
- **Shooter speed control:** feedforward + PID for a flywheel shooter, with the wheel speed read through an OctoQuad.
- **Panels** (https://panels.bylazar.com) for live telemetry, graphs and tuning without recompiling.
- **Engineering studies** written for high school students: shooter PID, shot trajectory, and PathMaker vs Pedro Pathing.
- **Bay-b bot reference code**: the coach's test robot, set up for Pedro Pathing 3.

It is used to prepare and explain changes that the students then bring into their team repo ([bayrobotics/3rd-Gear-26-27](https://github.com/bayrobotics/3rd-Gear-26-27)).

> **Status (3 Nov 2026):** The TeamCode compiles against the FTC SDK 11.2.1 and Panels libraries, checked with `javac`. A full Gradle build has not been run. Nothing has run on a robot yet. The next planned step is to move to SDK 12.0 and merge the bay-b bot code. See the roadmap below.

## Layout

```
├── FtcRobotController/          Android Studio project (open THIS folder in Android Studio)
│   ├── FtcRobotController/      FTC SDK module (unchanged from FIRST, v11.2.1)
│   ├── TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
│   │   ├── pathmaker/           PathMaker: PathManager (control), PathDetails (paths),
│   │   │                        PathMakerStateMachine (sequencing), RobotPoseSimulation
│   │   ├── hw/                  Hardware: DriveTrain, Shooter, VelocityPidController, ExtraStuff,
│   │   │                        MyIMU, VisionSystem
│   │   ├── op/                  OpModes: TeleOpMode, AutoOp_* (red/blue, close/far), ShooterTuning,
│   │   │                        RobotPose (odometry)
│   │   ├── configuration/       RobotConfig, GameConfig, ShooterConfig (all live-editable in Panels)
│   │   └── panelsdemo/          Small Java and Kotlin Panels examples
│   └── _panels_backup/          Gradle files from before Panels was added
├── docs/                        Studies and guides (see below)
│   ├── shooter_sim/             Laptop simulations (plain Java, no Android needed)
│   └── shooter_img/             Figures used by the documents
└── BabyBot/teamcode/            Bay-b bot code, for reference (Pedro Pathing 3, Ivy, AutoTune
                                 procedures). Not part of the build yet
```

## Documents

| Document | What it covers |
|---|---|
| [docs/SHOOTER-PID.md](docs/SHOOTER-PID.md) | Why the shooter needs closed-loop speed control. Feedforward + PID, measuring speed with a REV Through Bore Encoder on the OctoQuad, tuning steps, and simulated four-ball bursts |
| [docs/SHOOTER-TRAJECTORY.md](docs/SHOOTER-TRAJECTORY.md) | Best launch angle and ball speed for the leaning triangular goal, how much error a shot can tolerate, fixed vs adjustable hood, ball speed → wheel RPM |
| [docs/PATHING-PEDRO-VS-PATHMAKER.md](docs/PATHING-PEDRO-VS-PATHMAKER.md) | Review of the bay-b bot merge plan, how PathMaker and Pedro Pathing 3 work, pros and cons, and a roadmap for PathMaker 2 |
| [docs/PATHING-BENCHMARK-PLAN.md](docs/PATHING-BENCHMARK-PLAN.md) | Test plan for comparing the two pathing libraries on the bay-b bot |
| [docs/PANELS-SETUP.md](docs/PANELS-SETUP.md) | How Panels and Kotlin were added to the Gradle build, and why these versions |

## Getting started

1. Install Android Studio **Narwhal 3 Feature Drop or later** (required by AGP 8.13.2).
2. Clone this repo and open the `FtcRobotController/` folder in Android Studio. Let Gradle sync. If Android Studio offers to "upgrade" or "downgrade" the Android Gradle Plugin, **decline**.
3. Connect to the Control Hub's Wi-Fi and run the `TeamCode` configuration.
4. Open Panels at http://192.168.43.1:8001 while connected to the robot.

Versions: FTC SDK 11.2.1, Gradle 9.1.0, AGP 8.13.2, Kotlin plugin 2.4.10, Panels `com.bylazar:fullpanels:1.0.12`.

### Robot configuration names used by the code

| Name | Device |
|---|---|
| `front_left`, `front_right`, `back_left`, `back_right` | Drive motors |
| `intake_motor`, `outtake_motor` | Intake and shooter motors |
| `intake_servo` | Intake servo |
| `octoquad` | OctoQuad (shooter Through Bore encoder on channel 0, set in `ShooterConfig.encoderChannel`) |
| `imu` | Control Hub IMU |
| `Webcam 1` | Webcam (optional) |

### Running the laptop simulations

From `docs/shooter_sim`, with any JDK 17+:

```
javac -d out -sourcepath ../../FtcRobotController/TeamCode/src/main/java ShooterSim.java
java -cp out ShooterSim
javac -d out TrajectorySim.java
java -cp out TrajectorySim          # takes a few minutes
```

## Roadmap

1. Move to **FTC SDK 12.0** (the BIOBUZZ release, needed by Pedro Pathing 3). This also requires updating the AprilTag code for the new detection types.
2. Merge the bay-b bot code. Pedro's OctoQuad localizer uses ports 0 and 1, so the shooter encoder moves to another channel.
3. Allow either PathMaker or Pedro Pathing to drive the same routes, and run the benchmark plan.
4. PathMaker 2: non-blocking update, pluggable localizer, paths as data with alliance mirroring, braking from measured stopping distance, and a short self-measuring tuning OpMode.

## History

Earlier versions of PathMaker (2023–2025: field-centric driving, AprilTag navigation, Roomba-style tests) are kept in this repo's history:

- tag **`legacy-2025`**: the last state before this workspace was imported;
- branch **`Roomba`**: an earlier experiment branch.

## License

- PathMaker and the TeamCode in this repo: MIT, © bayrobotics.org (see the file headers).
- `FtcRobotController/` SDK: FIRST's license, in `FtcRobotController/LICENSE`.
- `BabyBot/teamcode/pedro/procedures`: AutoTune procedures copied from the Pedro Pathing Quickstart (github.com/Pedro-Pathing/Quickstart). The Pedro Pathing library is BSD 3-Clause.
