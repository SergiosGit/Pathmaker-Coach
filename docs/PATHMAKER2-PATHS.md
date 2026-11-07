# Writing autonomous paths with PathMaker 2

This guide is for writing and changing autonomous routes. You don't need to know how
PathMaker 2 works inside to use it; that is explained in `pathmaker2/PathMaker2.java` when you
want to dig in.

## 1. The idea

A route is a list of steps, read top to bottom. Each step either **moves the robot** or
**does something** (shooter, intake, servo, wait). PathMaker 2 drives each move as fast as the
robot can while still stopping on the spot, using numbers measured on our robot by the
calibration page. There are no ramps, powers or delays to tune.

```java
public static Route close() {
    return Route.start(CLOSE_START)
            .shooter(3000)            // start spinning up; keeps running during the moves
            .to(52, 17, 90)           // drive here and stop
            .waitForShooter(2)        // wait until the wheel is at speed (at most 2 s)
            .feed(5)                  // open the gate and run the intake for 5 s
            .shooter(0)
            .to(12, 31, 175)          // turns to 175° while it drives
            .build();
}
```

A move ends when the robot has stopped within ½ inch and 1° of the target. Each move also has
a timeout (4 s unless you set one), so a robot that gets blocked still finishes the route.

## 2. Coordinates

We use the field coordinates that Pedro Pathing and the Panels field view use:

- The field is 144 × 144 inches. (0, 0) is the bottom-left corner of the field picture in
  Panels; check once which real corner that is on the BIOBUZZ field and write it here.
- **x** goes right and **y** goes up in that picture.
- **Heading** is in degrees, measured from the +x direction, **counter-clockwise is positive**.
  90° faces up the field (+y), 180° faces left (−x).

> Watch out: our old PathMaker used a different system: it started at (0, 0, 0) wherever the
> robot was, with y forward, x to the right and clockwise turns positive. A turn of −85° there
> is +85° (to the left) here.

The start pose is the one measurement you must make on a real field: put the robot where it
starts, measure the centre of the robot from the left and bottom walls, and note its heading.
Write it once at the top of `Routes.java`:

```java
static final Pose CLOSE_START = new Pose(56, 6, 90);   // measured 2026-11-xx
```

Tip: write the other points relative to the start while you are figuring a route out, then
replace them with plain numbers once it works:

```java
.to(CLOSE_START.x - 4, CLOSE_START.y - 11, 90)   // 4 in left, 11 in back
```

## 3. Blue and red: write it once

Write every route **for blue**. The red version is made automatically by mirroring it across
the centre line of the field: x becomes 144 − x and the heading becomes 180° − heading. A move
4 inches to the left on blue is 4 inches to the right on red, and a turn to the left becomes
a turn to the right.

> Check this against the BIOBUZZ field drawing once: if the red side is a mirror image of the
> blue side, mirroring is right. (If the field were rotated instead, we'd change one line in
> `Route.mirrored()`.)

There is one small OpMode per start position and colour, so the drive team still just picks
"Close Blue" from the Driver Station list:

```java
@Autonomous(name = "Close Blue")
public class AutoCloseBlue extends AutoBase {
    public AutoCloseBlue() { super(Alliance.BLUE, Routes.close()); }
}
```

`AutoCloseRed` is the same with `Alliance.RED`. That is the only thing that differs; you
never copy a route.

## 4. The steps

### Moving

| Step | What it does |
|---|---|
| `to(x, y, heading)` | Drive straight to the point and stop. The heading changes smoothly during the move. |
| `through(x, y, heading)` | Drive towards the point but don't stop: the next move starts about 3 in before it. Use this for corners you just need to get around. |
| `turnTo(heading)` | Turn in place. |
| `speed(0.5)` | Use this fraction of top speed for the moves after it, e.g. near a wall or while intaking. `speed(1)` goes back to full speed. |
| `timeout(3)` | The timeout in seconds for the next move only. |

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
| `shooter(rpm)` | Set the shooter speed in real RPM (0 = off). It keeps the speed with the PID during the steps that follow. |
| `waitForShooter(maxSeconds)` | Wait until the shooter is at speed. |
| `intake(power)` | Set the intake power, −1 to 1. |
| `servo(position)` | Move the intake servo, 0 to 1. |
| `feed(seconds)` | Shoot: servo to its feed position, run the intake, wait, then servo back. The positions are set once in the robot config. |
| `wait(seconds)` | Wait; the robot holds its position. |

Things you set stay set until you change them. They happen at the moment the step is reached:
`intake(0.8)` before a `to(...)` runs the intake during that drive.

## 5. Making a route: step by step

1. **Sketch it** on the field drawing with the start pose and the points where the robot must
   stop. Fewer points is better: PathMaker 2 drives straight between them.
2. **Write it** in `Routes.java` with `to(...)` for each stop.
3. **First run at low speed:** put `speed(0.4)` at the top. Watch the robot and the
   telemetry, which shows the step number, the target and how far off the robot is.
4. **Fix the points**, not the controller: if the robot stops 2 in short of where you want it,
   the point is wrong (or the start pose is). Move the point.
5. **Speed up:** remove `speed(0.4)`. Keep a lower speed only where it matters, such as close
   to a wall or while collecting.
6. **Shorten the waits:** replace fixed waits with `waitForShooter`, and shorten `feed` to
   just long enough for all the balls.
7. **Check red:** run the red OpMode once on the red side.

If something looks wrong with the *driving itself* (it overshoots, wobbles or crawls at the
end), that's not the route: tell the coach, it's a calibration or PathMaker issue.

## 6. Example: our close route, from the old PathMaker

This is what `AutoOp_BlueClose` (paths B11 to B15) does, written the new way. In the old
version each step's speed was set with `powerScaling` and its end with `pathTime_ms`. Steps
that should not move used a delay longer than the step. None of that is needed now.

| Old | What it does | New |
|---|---|---|
| B11 | 4 in to the left, shooter on | `.shooter(RPM).to(S.x - 4, S.y, 90)` |
| B12 | back up 11 in | `.to(S.x - 4, S.y - 11, 90)` |
| B13 | wait 5 s with the intake servo at 0 (shoot) | `.waitForShooter(2).feed(5)` |
| B14 | turn 85° to the left, shooter off | `.shooter(0).turnTo(175)` |
| B15 | to 40 in left and 25 in up the field | `.to(S.x - 44, S.y + 14, 175)` |

```java
static final Pose S = CLOSE_START;
static final double RPM = 3000;   // pick from the shooter tuning (docs/SHOOTER-PID.md)

public static Route close() {
    return Route.start(S)
            .shooter(RPM)
            .to(S.x - 4, S.y, 90)
            .to(S.x - 4, S.y - 11, 90)
            .waitForShooter(2)
            .feed(5)
            .shooter(0)
            .turnTo(175)
            .to(S.x - 44, S.y + 14, 175)
            .build();
}
```

Things found while converting:

- **Red and blue close were the same.** R11 to R15 are identical to B11 to B15, so one of them
  goes the wrong way: on red the robot should move right and turn right. Mirroring fixes this.
- **In the old code the autonomous never ran the shooter.** `Extra_Stuff` was called with the
  shooter and intake values swapped, so the shooter got 0 and the intake 0.95.
- **85° instead of 90°.** The old PathMaker stopped turning up to 10° early, which may be why
  85° was used. PathMaker 2 stops within 1°, so use the angle you actually want (probably 180°).
- **B11 and B12 can be one move** (`to(S.x - 4, S.y - 11, 90)`) unless the robot has to clear
  something first.
