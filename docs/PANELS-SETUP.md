# Panels + Kotlin setup

**Repo:** `sw/FtcRobotController` — FTC SDK 11.2.1, Gradle wrapper 9.1.0, AGP 8.13.2

> Since 3 Nov 2026 the project is on FTC SDK 12.0. 12.0 kept Gradle 9.1.0 and AGP 8.13.2, so everything below still applies.
**Robot:** REV Control Hub · **Set up:** 2026-08-18

## What was changed

| File | Change |
|---|---|
| `build.gradle` (root) | Added `org.jetbrains.kotlin.android` 2.4.10 to the `plugins {}` block (`apply false`), and added the Panels maven repo `https://mymaven.bylazar.com/releases` to `allprojects.repositories`. |
| `TeamCode/build.gradle` | Applied `org.jetbrains.kotlin.android`; set `compileSdk 35`; added `kotlin { compilerOptions { jvmTarget = JVM_1_8 } }`; added `implementation 'com.bylazar:fullpanels:1.0.12'`. |
| `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/panelsdemo/` | New `PanelsKotlinDemo.kt`, `PanelsDemoConfig.kt`, `PanelsJavaDemo.java`. |

Originals are backed up in `sw/FtcRobotController/_panels_backup/`.

Nothing in `FtcRobotController/`, `build.common.gradle` or `build.dependencies.gradle` was touched,
so pulling FTC SDK updates stays clean. `minSdk` (24) and `targetSdk` (28) are unchanged —
only the *compile* SDK moved, which Panels requires.

Why the dependency lives in `TeamCode` and not in the shared `build.dependencies.gradle`: that shared
file is also applied by the `FtcRobotController` library module, which compiles against API 30 and would
fail the Panels AAR's compileSdk requirement. `TeamCode` is the module that actually becomes the APK,
so putting it there is both sufficient and minimal.

## Version matrix (why these numbers)

- Gradle wrapper 9.1.0 + AGP 8.13.2 (shipped by FTC SDK 11.2.1)
- Kotlin Gradle plugin **2.4.10** — the only current KGP line whose support window covers
  Gradle 7.6.3–9.5.0 *and* AGP 8.5.2–9.1.0.
- Panels `com.bylazar:fullpanels:1.0.12` (latest release).
- `jvmTarget` is set through `kotlin { compilerOptions { … } }`; the old `kotlinOptions` block
  was removed in the Kotlin 2.x plugin.

## Accessing Panels

1. Join the Control Hub's WiFi network from your laptop.
2. Open <http://192.168.43.1:8001>  (a phone RC would be `192.168.49.1:8001`).

The two demo OpModes appear on the Driver Hub under group **Panels**. Both are hardware-free
and run with no robot configuration, so they are the fastest way to prove the toolchain works.

## Team code layout convention

Each student gets one package under `teamcode`, named in lowercase to match its folder:

```
teamcode/
  daksh_summer26/
    DriveTrain.java
    ArmMovement.java
    mechanisms/TestBench1.java     → package …teamcode.daksh_summer26.mechanisms
  panelsdemo/
    PanelsKotlinDemo.kt
```

The package declaration must match the folder path exactly. This keeps two students from
colliding when they both write a `DriveTrain`, and stops Android Studio from flagging the file.

## Do NOT accept Android Studio's AGP upgrade prompt

Studio will offer to run the **AGP Upgrade Assistant** (AGP 8.13.2 → 9.x). Decline it.

It was accepted once on 2026-08-18 and reverted. What it did:

- bumped AGP to 9.1.1 and the Gradle wrapper to 9.3.1,
- moved `targetSdkVersion 28` out of `defaultConfig` into `lint {}` / `testOptions {}`
  in `FtcRobotController/build.gradle`,
- deleted `packagingOptions { pickFirst '**/*.so' }` from `build.common.gradle`,
- appended eleven `android.*=false` flags to `gradle.properties`.

Why that is bad here: those eleven flags exist only to make AGP 9 behave like AGP 8, and the
opt-out is removed in AGP 10. Meanwhile the FTC SDK still pins `ndkVersion 21.3.6528147` and
`compileSdkVersion 30`, while AGP 9 requires Build Tools 36 and defaults to NDK r28c. And it
edits `build.common.gradle` and `FtcRobotController/build.gradle` — the two files we keep
byte-identical to upstream so the next season's SDK merge stays trivial.

Stay on what FIRST ships and tests: **AGP 8.13.2 + Gradle 9.1.0**.

To recover if it happens again (note: `git checkout` fails on this mount, use `git show`):

```bash
cd sw/FtcRobotController
for f in build.common.gradle FtcRobotController/build.gradle \
         gradle.properties gradle/wrapper/gradle-wrapper.properties; do
  git show "HEAD:$f" > "$f"
done
# then set AGP back to 8.13.2 in the root build.gradle plugins block
```

A healthy `git status` shows exactly two modified files — `build.gradle` and
`TeamCode/build.gradle` — plus the untracked student and demo packages.
