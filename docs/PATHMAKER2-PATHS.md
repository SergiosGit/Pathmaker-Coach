# Writing autonomous paths with PathMaker 2

This guide is for writing and changing autonomous routes. You don't need to know how
PathMaker 2 works inside to use it; that is explained in `pathmaker2/PathMaker2.java` when you
want to dig in.

All routes live in one file, `auto/Routes.java`. That is the only file you need to change for
autonomous.

## 1. The idea

A route is a list of steps, read top to bottom. Each step either **moves the robot** or
**does something** (shooter, intake, servo, wait). PathMaker 2 drives each move as fast as the
robot can while still stopping on the spot, using numbers measured on our robot by the
calibration page. There are no ramps, powers or delays to tune.

```java
public static Route close() {
    FieldPose s = CLOSE_START;
    return Route.start(s)
            .shooter(4500)               // start spinning up; keeps running during the moves
            .to(s.x - 4, s.y - 11, 90)   // drive here and stop
            .waitForShooter(2)           // wait until the wheel is at speed (at most 2 s)
            .feed(5)                     // feed the balls into the shooter for 5 s
            .shooter(0)
            .to(s.x - 44, s.y + 14, 175) // turns to 175° while it drives
            .build();
}
```

A move ends when the robot has stopped within ½ inch and 1° of the target. Each move also has
a timeout (4 s unless you set one), so a robot that gets blocked still finishes the route.

> Before the first autonomous run, the robot must be calibrated: run the **Calibrate** OpMode
> and follow the page at http://192.168.43.1:8090 (see docs/CALIBRATION.md). Until then the
> autonomous shows "NOT CALIBRATED" during INIT and drives with guessed numbers. It may stop a
> few degrees short of a turn and wait for the timeout.

## 2. Coordinates

We use the field coordinates that Pedro Pathing and the Panels field view use:

- The field is 144 × 144 inches. (0, 0) is the bottom-left corner of the field picture in
  Panels; check once which real corner that is on the BIOBUZZ field and write it here.
- **x** goes right and **y** goes up in that picture.
- **Heading** is in degrees, measured from the +x direction, **counter-clockwise is positive**.
  90° faces up the field (+y), 180° faces left (−x), 0° faces right (+x).

> Watch out: our old PathMaker used a different system: it started at (0, 0, 0) wherever the
> robot was, with y forward, x to the right and clockwise turns positive. A turn of −85° there
> is +85° (to the left) here.

The start pose is the one measurement you must make on a real field: put the robot where it
starts, measure the centre of the robot from the left and bottom walls, and note its heading.
Write it once at the top of `Routes.java`:

```java
public static final FieldPose CLOSE_START = new FieldPose(56, 24, 90);   // measured 2026-11-xx
```

Tip: write the other points relative to the start while you are figuring a route out, then
replace them with plain numbers once it works:

```java
.to(s.x - 4, s.y - 11, 90)   // 4 in left, 11 in back
```

## 3. Blue and red: write it once

Write every route **for blue**. The red route is made from it automatically. How depends on
how the BIOBUZZ field is built, which you set with one line at the top of `Routes.java`.
Look at the official field drawing: is the red half the blue half seen in a **mirror**, or the
blue half **turned half way round**?

### Case 1: red is a mirror image of blue

```java
public static final FieldSymmetry SYMMETRY = FieldSymmetry.MIRROR;
```

Every point is flipped across the line down the middle of the field, x = 72:

```
x       → 144 − x
y       → y              (unchanged)
heading → 180° − heading
```

So "left" on blue is "right" on red, and a turn to the left becomes a turn to the right. The
close route from section 1, written for blue, drives this on red:

```java
// blue (what you write)                 // red (made automatically)
Route.start(56, 24, 90)                  // start(88, 24, 90)
        .to(52, 13, 90)                  //  .to(92, 13, 90)    4 in RIGHT, 11 in back
        .to(12, 38, 175)                 //  .to(132, 38, 5)    turns RIGHT to 5°
```

### Case 2: red is blue turned half way round

```java
public static final FieldSymmetry SYMMETRY = FieldSymmetry.ROTATE;
```

Every point is turned 180° around the centre of the field, (72, 72):

```
x       → 144 − x
y       → 144 − y
heading → heading + 180°
```

The red robot starts at the top of the field, facing down, and does exactly what the blue robot
does as seen from its own driver station: "left" is still left and a left turn is still a left
turn. The same blue route drives this on red:

```java
// blue (what you write)                 // red (made automatically)
Route.start(56, 24, 90)                  // start(88, 120, -90)
        .to(52, 13, 90)                  //  .to(92, 131, -90)   4 in left, 11 in back
        .to(12, 38, 175)                 //  .to(132, 106, -5)   turns left to -5°
```

### What does not change

Shooter, intake, servo, speed, timeout and wait steps are the same for both colours. The
start pose is transformed like any other point, so you only measure the blue one.

This is all the code there is for it (`auto/FieldSymmetry.java`):

```java
public FieldPose apply(FieldPose p) {
    switch (this) {
        case MIRROR: return new FieldPose(144 - p.x, p.y, wrap(180 - p.heading));
        default:     return new FieldPose(144 - p.x, 144 - p.y, wrap(p.heading + 180));
    }
}
```

Check it once on the field: run the red OpMode on the red side, at low speed.

### One OpMode per start and colour

The drive team still just picks "Close Blue" from the Driver Station list. Each OpMode is a
three-line file that names the colour and the route; you never copy a route:

```java
@Autonomous(name = "Close Blue", group = "Auto")
public class AutoCloseBlue extends AutoBase {
    public AutoCloseBlue() { super(Alliance.BLUE, Routes.close()); }
}
```

`AutoCloseRed` is the same with `Alliance.RED`. For a new start position, add a route to
`Routes.java` and two of these files.

TeleOp uses the same symmetry: after autonomous, "forward" on the left stick points away from
that alliance's drivers (`BLUE_DRIVER_FORWARD` in `Routes.java`).

## 4. The steps

### Moving

| Step                     | What it does                                                                                                                         |
| ------------------------ | ------------------------------------------------------------------------------------------------------------------------------------ |
| `to(x, y, heading)`      | Drive straight to the point and stop. The heading changes smoothly during the move.                                                  |
| `through(x, y, heading)` | Drive towards the point but don't stop: the next move starts about 3 in before it. Use this for corners you just need to get around. |
| `turnTo(heading)`        | Turn in place.                                                                                                                       |
| `speed(0.5)`             | Use this fraction of top speed for the moves after it, e.g. near a wall or while intaking. `speed(1)` goes back to full speed.       |
| `timeout(3)`             | The timeout in seconds for the next move only.                                                                                       |

A row of `through` points followed by a `to` drives a smooth chain, for example around an
obstacle:

```java
.through(40, 40, 90)
.through(40, 80, 90)
.to(60, 100, 45)
```

### Doing things

| Step | What it does |
|---|---|
| `shooter(rpm)` | Set the shooter speed in real RPM (0 = off). It keeps the speed with the PID during the steps that follow. `ShooterConfig.autoTargetRpm` is the usual value. |
| `waitForShooter(maxSeconds)` | Wait until the shooter is at speed. |
| `intake(power)` | Set the intake power, −1 to 1. |
| `servo(position)` | Move the intake servo, 0 to 1. |
| `feed(seconds)` | Shoot: servo to its feed position and run the intake, wait, then servo back and the intake as it was. The positions and power are in `RobotConfig.Intake` (the same as TeleOp's dpad left/right and left trigger). |
| `pause(seconds)` | Wait; the robot holds its position. (Not called `wait`: Java already has a `wait()` that would be used instead and crash.) |

Shooter, intake and servo steps happen at the moment the step is reached and stay set until
you change them: `intake(0.8)` before a `to(...)` runs the intake during that drive, and
between two `through` points it starts when the robot heads for the second one.
`waitForShooter`, `feed` and `pause` stop the robot; it holds its position while they run.

## 5. Making a route: step by step

1. **Sketch it** on the field drawing with the start pose and the points where the robot must
   stop. Fewer points is better: PathMaker 2 drives straight between them.
2. **Write it** in `Routes.java` with `to(...)` for each stop.
3. **First run at low speed:** keep `speed(0.5)` at the top. Watch the robot and the
   telemetry, which shows the step number, the target and how far off the robot is.
4. **Fix the points**, not the controller: if the robot stops 2 in short of where you want it,
   the point is wrong (or the start pose is). Move the point.
5. **Speed up:** remove `speed(0.5)`. Keep a lower speed only where it matters, such as close
   to a wall or while collecting.
6. **Shorten the waits:** shorten `feed` to just long enough for all the balls.
7. **Check red:** run the red OpMode once on the red side.

If something looks wrong with the *driving itself* (it overshoots, wobbles or crawls at the
end), that's not the route: tell the coach, it's a calibration or PathMaker issue.

## 6. Example: our close route, from the old PathMaker

This is what `AutoOp_BlueClose` (paths B11 to B15) did, written the new way. In the old
version each step's speed was set with `powerScaling` and its end with `pathTime_ms`. Steps
that should not move used a delay longer than the step. None of that is needed now.

| Old | What it does | New |
|---|---|---|
| B11 | 4 in to the left, shooter on | `.shooter(RPM).to(s.x - 4, s.y, 90)` |
| B12 | back up 11 in | `.to(s.x - 4, s.y - 11, 90)` |
| B13 | wait 5 s with the intake servo at 0 (shoot) | `.waitForShooter(2).feed(5)` |
| B14 | turn 85° to the left, shooter off | `.shooter(0).turnTo(175)` |
| B15 | to 40 in left and 25 in up the field | `.to(s.x - 44, s.y + 14, 175)` |

```java
public static Route close() {
    FieldPose s = CLOSE_START;
    return Route.start(s)
            .speed(0.5)                         // first tests: raise to 1 once it drives well
            .shooter(ShooterConfig.autoTargetRpm)
            .to(s.x - 4, s.y, 90)               // B11: 4 in left
            .to(s.x - 4, s.y - 11, 90)          // B12: back 11 in
            .waitForShooter(2)
            .feed(5)                            // B13: shoot
            .shooter(0)
            .turnTo(175)                        // B14: 85° left (the old -85°). 180° meant?
            .to(s.x - 44, s.y + 14, 175)        // B15
            .build();
}
```

The far route (B21 to B28) is converted the same way in `Routes.far()`.

Things found while converting:

- **Red and blue close were the same.** R11 to R15 are identical to B11 to B15, so one of them
  went the wrong way. Making red from blue fixes this.
- **In the old code the autonomous never ran the shooter.** `Extra_Stuff` was called with the
  shooter and intake values swapped, so the shooter got 0 and the intake 0.95.
- **85° instead of 90°.** The old PathMaker stopped turning up to 10° early, which may be why
  85° was used. PathMaker 2 stops within 1°, so use the angle you actually want (probably 180°
  here, and 0° in the far route).
- **B11 and B12 can be one move** (`to(s.x - 4, s.y - 11, 90)`) unless the robot has to clear
  something first.
