package frc.robot.Commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.FieldConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Turret;
import frc.robot.util.dashboard.TurretUtil;

public class GameCommands {

    // Robot is rotated so the target lands here when rotating into range.
    private static final double kTurretRangeCenterDeg = 135.0;

    // Turret must be within this tolerance (degrees) before the shooter fires.
    private static final double kTurretAlignToleranceDeg = 5;

    private final Shooter shooter;
    private final Turret turret;
    private final Spindexer spindexer;
    private final CommandSwerveDrivetrain drivetrain;
    private final CommandXboxController xboxController;

    private final SwerveRequest.FieldCentricFacingAngle rotateInPlaceRequest = new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public GameCommands(Shooter shooter, Turret turret, CommandSwerveDrivetrain drivetrain, Spindexer spindexer,
            CommandXboxController xboxController) {
        this.shooter = shooter;
        this.turret = turret;
        this.spindexer = spindexer;
        this.drivetrain = drivetrain;
        this.xboxController = xboxController;

        rotateInPlaceRequest.HeadingController.setPID(2, 0, 0.1);
        rotateInPlaceRequest.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Registers all game commands as PathPlanner named commands.
     * Call this in RobotContainer BEFORE CommandSwerveDrivetrain is constructed
     * (i.e., before AutoBuilder.configure() is called inside
     * configureAutoBuilder()).
     */
    public void registerNamedCommands() {
        NamedCommands.registerCommand("ShootAtHub", shootAtHubCommand());
        NamedCommands.registerCommand("AutoShootAtHub", autoShootAtHubCommand());
        NamedCommands.registerCommand("PassLeft", passLeftCommand());
        NamedCommands.registerCommand("PassRight", passRightCommand());
    }

    /**
     * Rotates the robot in place until the given target falls within the turret's
     * allowed range (ShotSolution.isValid). Ends immediately if already in range.
     */
    public Command rotateIntoRangeCommand(TurretUtil.TargetType target,
            Supplier<Translation2d> targetXY) {
        return drivetrain.applyRequest(() -> {
            Translation2d tgt = targetXY.get();
            Translation2d robot = drivetrain.getState().Pose.getTranslation();

            double fieldAngleDeg = Math.toDegrees(
                    Math.atan2(tgt.getY() - robot.getY(), tgt.getX() - robot.getX()));

            // Rotate so the target lands at 135° in the turret frame (center of [90°,
            // 180°])
            Rotation2d goalHeading = Rotation2d.fromDegrees(fieldAngleDeg - kTurretRangeCenterDeg);

            return rotateInPlaceRequest
                    .withVelocityX(0)
                    .withVelocityY(0)
                    .withTargetDirection(goalHeading);
        })
                .until(() -> {
                    var pose = drivetrain.getState().Pose;
                    TurretUtil.ShotSolution solution = TurretUtil.computeShotSolution(pose, target);
                    return solution.isValid;
                })
                .withTimeout(4)
                .withName("RotateIntoTurretRange-" + target);
    }

    /**
     * Auto-aims the turret at the target, waits until aligned within tolerance,
     * then spins up the shooter using the lookup-table speed from ShotSolution.
     * Runs until the command is cancelled.
     */
    public Command aimAndShootCommand(TurretUtil.TargetType target) {
        return Commands.parallel(
                // Continuously track the target with yaw-rate feedforward
                turret.autoAimCommand(
                        () -> drivetrain.getState().Pose,
                        () -> target,
                        () -> drivetrain.getState().Speeds.omegaRadiansPerSecond),
                // Wait for the turret to settle, then run the shooter at the lookup-table speed
                Commands.waitUntil(() -> Math.abs(turret.getErrorDegrees()) < kTurretAlignToleranceDeg)
                        .andThen(Commands.run(() -> {
                            var pose = drivetrain.getState().Pose;
                            TurretUtil.ShotSolution solution = TurretUtil.computeShotSolution(pose, target);
                            if (solution.isValid) {
                                spindexer.clockwise(0.5);
                                shooter.setVelocity(solution.shooterSpeedRPS);
                                xboxController.setRumble(RumbleType.kBothRumble, 0);
                            } else {
                                // System.out.println("");
                                xboxController.setRumble(RumbleType.kBothRumble, 1);
                            }
                        }, shooter, spindexer))
                        .finallyDo(__ -> {
                            shooter.stop();
                            spindexer.stop();
                            xboxController.setRumble(RumbleType.kBothRumble, 0);
                        }))
                .withName("AimAndShoot-" + target);
    }

    public Command shootWithoutAngleCheckCommand(TurretUtil.TargetType target) {
        return Commands.run(() -> {
            var pose = drivetrain.getState().Pose;
            TurretUtil.ShotSolution solution = TurretUtil.computeShotSolution(pose, target);
            spindexer.clockwise(0.5);
            shooter.setVelocity(solution.shooterSpeedRPS);
            xboxController.setRumble(RumbleType.kBothRumble, 0);
            
        }, shooter, spindexer)
                .finallyDo(__ -> {
                    shooter.stop();
                    spindexer.stop();
                    xboxController.setRumble(RumbleType.kBothRumble, 0);
                })
                .withName("AimAndShoot-" + target);
    }

    public Command shootAtSpeedWithoutAngleCheckCommand(double RPS) {
        return Commands.run(() -> {
            spindexer.clockwise(0.5);
            shooter.setVelocity(RPS);
        }, shooter, spindexer)
                .finallyDo(__ -> {
                    shooter.stop();
                    spindexer.stop();
                    xboxController.setRumble(RumbleType.kBothRumble, 0);
                });
    }

    /**
     * Rotates the robot if the hub is outside the turret's range, auto-aims the
     * turret using ShotSolution, then spins the shooter at the lookup-table speed.
     * Runs until cancelled — use this as a PathPlanner named command "ShootAtHub"
     * to shoot all balls during the auto period.
     */
    public Command shootAtHubCommand() {
        // getAllianceHub() is evaluated lazily (at command execution time) so the
        // alliance is already set by the time the command actually runs.
        Supplier<Translation2d> hub = () -> FieldConstants.getAllianceHub().toTranslation2d();

        return Commands.sequence(
                rotateIntoRangeCommand(TurretUtil.TargetType.HUB, hub).withTimeout(4),
                aimAndShootCommand(TurretUtil.TargetType.HUB)).withName("ShootAtHub");
    }

    /**
     * Auto-only command: rotates into range, aims turret at the hub, then waits
     * 2 seconds for vision (Limelight) to refine the robot pose before firing.
     * This ensures odometry is corrected by AprilTag measurements before committing
     * to a shot. Runs until cancelled.
     */
    public Command autoShootAtHubCommand() {
        Supplier<Translation2d> hub = () -> FieldConstants.getAllianceHub().toTranslation2d();

        return Commands.sequence(
                rotateIntoRangeCommand(TurretUtil.TargetType.HUB, hub),
                // Start aiming while we wait for vision to settle
                Commands.parallel(
                        turret.autoAimCommand(
                                () -> drivetrain.getState().Pose,
                                () -> TurretUtil.TargetType.HUB,
                                () -> drivetrain.getState().Speeds.omegaRadiansPerSecond),
                        Commands.sequence(
                                // Wait 2 seconds for Limelight vision to refine pose
                                Commands.waitSeconds(2.0),
                                // Then wait for turret alignment and shoot
                                Commands.waitUntil(() -> Math.abs(turret.getErrorDegrees()) < kTurretAlignToleranceDeg),
                                Commands.run(() -> {
                                    var pose = drivetrain.getState().Pose;
                                    TurretUtil.ShotSolution solution = TurretUtil.computeShotSolution(
                                            pose, TurretUtil.TargetType.HUB);
                                    if (solution.isValid) {
                                        spindexer.clockwise(0.5);
                                        shooter.setVelocity(solution.shooterSpeedRPS);
                                    }
                                }, shooter, spindexer)
                                        .finallyDo(__ -> {
                                            shooter.stop();
                                            spindexer.stop();
                                        }))))
                .withName("AutoShootAtHub");
    }

    /**
     * Same structure as shootAtHubCommand but targets the left pass location.
     */
    public Command passLeftCommand() {
        Supplier<Translation2d> target = () -> FieldConstants.leftPassTarget.getTranslation().toTranslation2d();

        return Commands.sequence(
                rotateIntoRangeCommand(TurretUtil.TargetType.LEFT_PASS, target),
                aimAndShootCommand(TurretUtil.TargetType.LEFT_PASS)).withName("PassLeft");
    }

    /**
     * Same structure as shootAtHubCommand but targets the right pass location.
     */
    public Command passRightCommand() {
        Supplier<Translation2d> target = () -> FieldConstants.rightPassTarget.getTranslation().toTranslation2d();

        return Commands.sequence(
                rotateIntoRangeCommand(TurretUtil.TargetType.RIGHT_PASS, target),
                aimAndShootCommand(TurretUtil.TargetType.RIGHT_PASS)).withName("PassRight");
    }
}
