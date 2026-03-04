# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Structure

All robot code lives under `Design 1/` — this is the Gradle project root. Source code is at `Design 1/src/main/java/frc/robot/`.

## Build Commands

Run all commands from inside `Design 1/`:

```bash
./gradlew build          # Compile and check for errors
./gradlew deploy         # Build and deploy to RoboRIO (robot must be connected)
./gradlew simulateJava   # Run desktop simulation
./gradlew test           # Run unit tests
```

## Architecture Overview

This is a **2026 FRC Command-Based robot** using WPILib 2.0 with CTRE Phoenix 6 (TalonFX motors, Pigeon2 gyro), REV Spark Max, and PathPlanner for autonomous.

### Entry Points
- `Robot.java` — TimedRobot lifecycle; enables Phoenix SignalLogger; links driver station
- `RobotContainer.java` — constructs all subsystems, registers default commands, configures button bindings, and builds the autonomous chooser

### Subsystems (`subsystems/`)
| Subsystem | Key Detail |
|-----------|------------|
| `CommandSwerveDrivetrain` | Extends CTRE `TunerSwerveDrivetrain`. **Do not edit generated sections.** Integrates PathPlanner AutoBuilder, vision odometry, and buffered sensor reading. |
| `Shooter` | Dual TalonFX (IDs 41/42). Velocity control; uses projectile-motion math (42° angle, `Constants.ShooterConstants`) to compute RPS from distance. |
| `Turret` | Single TalonFX (ID 44), 15:1 gearing. Supports position/velocity control and `autoAim()` commands (HUB, LEFT_PASS, RIGHT_PASS). |
| `Intake` | TalonFX (ID 18). Simple duty-cycle intake/score commands. |
| `Spindexer` | Spark Max motors (IDs 40/52). Magazine feeder with clockwise/counterclockwise commands. |
| `Climber` | TalonFX (ID 19) in brake mode. Time-based climb (0.75 s, configurable in `Constants.ClimberConstants`). |
| `VisionSubsystem` | Limelight mounted on turret. MegaTag2 pose estimation with 4-sample median filter fed into drivetrain odometry. |

### Vision / Odometry Pipeline
The Limelight sits on the turret, so camera pose must be corrected for both turret rotation and robot heading before being added to odometry:

```
CameraPose → LimelightOnTurretUtils (turret angle + gyro yaw) → robot pose → addVisionMeasurement()
```

Turret angle is read from a `BufferedStatusSignal` interpolated at the vision timestamp for accuracy.

### Key Utility Classes
- `Constants.java` — all tuning values (CAN IDs, PIDs, geometry offsets, shooter physics)
- `TunerConstants.java` (generated) — swerve module hardware configuration; regenerate via CTRE Tuner X, do not hand-edit
- `FieldConstants.java` (generated) — field geometry
- `Telemetry.java` — publishes robot state to NetworkTables for logging/visualization
- `BufferedStatusSignal.java` — interpolates CTRE status signals at an arbitrary past timestamp
- `util/LimelightOnTurretUtils.java` — transforms camera pose to field-relative robot pose accounting for turret and robot rotation

### Autonomous
PathPlanner autos and paths live in `Design 1/src/main/deploy/pathplanner/`. The `AutoBuilder` is configured in `CommandSwerveDrivetrain` with holonomic PID constants (translation kP=10, rotation kP=7).

### NetworkTables / Tuning
Tunable values are published under keys like `Turret/kP`, `Drivetrain/useVision`. Toggle vision odometry at runtime via `SmartDashboard` key `Drivetrain/useVision`.

## Important Conventions
- All subsystem commands that move mechanisms must declare that subsystem as a requirement (`addRequirements()`).
- The `CommandSwerveDrivetrain` has a `// This class is auto-generated` section — only edit code outside the generated region.
- Limelight pose updates are gated: skipped when the robot is rotating fast or the estimated pose is unreasonably far from current odometry (see `VisionSubsystem.periodic()`).
