package frc.robot.Commands;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.generated.FieldConstants;
import frc.robot.util.dashboard.TurretUtil;

public class GameCommands {

    private static final double kTurretRangeCenterDeg = 135.0;
    private static final double kTurretAlignToleranceDeg = 2;

    private final Robot Robot;

    private final SwerveRequest.FieldCentricFacingAngle rotateInPlaceRequest =
            new SwerveRequest.FieldCentricFacingAngle()
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public GameCommands(Robot robot) {

        this.Robot = robot;

        rotateInPlaceRequest.HeadingController.setPID(2, 0, 0.1);
        rotateInPlaceRequest.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void registerNamedCommands() {

        NamedCommands.registerCommand(
                "ShootAtHub",
                shootAtHubCommand(Optional.empty()));

        NamedCommands.registerCommand(
                "AutoShootAtHub",
                autoShootAtHubCommand(Optional.empty()));

        NamedCommands.registerCommand(
                "PassLeft",
                passLeftCommand(Optional.empty()));

        NamedCommands.registerCommand(
                "PassRight",
                passRightCommand(Optional.empty()));
    }

    public Command rotateIntoRangeCommand(
            TurretUtil.TargetType target,
            Supplier<Translation2d> targetXY) {

        return Robot.Drivetrain.applyRequest(() -> {

            Translation2d tgt = targetXY.get();
            Translation2d robot = Robot.Drivetrain.getState().Pose.getTranslation();

            double fieldAngleDeg = Math.toDegrees(
                    Math.atan2(tgt.getY() - robot.getY(), tgt.getX() - robot.getX()));

            Rotation2d goalHeading =
                    Rotation2d.fromDegrees(fieldAngleDeg - kTurretRangeCenterDeg);

            return rotateInPlaceRequest
                    .withVelocityX(0)
                    .withVelocityY(0)
                    .withTargetDirection(goalHeading);

        }).until(() -> {

            var pose = Robot.Drivetrain.getState().Pose;

            TurretUtil.ShotSolution solution =
                    TurretUtil.computeShotSolution(pose, target);

            return solution.isValid;

        }).withTimeout(4)
                .withName("RotateIntoTurretRange-" + target);
    }

    public Command aimAndShootCommand(
            TurretUtil.TargetType target,
            Optional<GenericHID> controller) {

        return Commands.parallel(

                Robot.Turret.autoAimCommand(
                        () -> Robot.Drivetrain.getState().Pose,
                        () -> target,
                        () -> Robot.Drivetrain.getState().Speeds,
                        () -> Units.degreesToRadians(Robot.Drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble()),
                        controller),

                Commands.waitUntil(() ->
                        Math.abs(Robot.Turret.getErrorDegrees()) < kTurretAlignToleranceDeg)

                        .andThen(

                                Commands.parallel(

                                        Commands.run(() -> {

                                            var pose = Robot.Drivetrain.getState().Pose;

                                            TurretUtil.ShotSolution solution = TurretUtil.computeShotSolution(pose, target);

                                            SmartDashboard.putNumber("Shooter/TargetRPS", solution.shooterSpeedRPS);

                                            if (solution.isValid) {

                                                Robot.Spindexer.clockwise(Constants.Spindexer.kIndexingSpeed);
                                                Robot.Shooter.setVelocity(solution.shooterSpeedRPS);

                                                // controller.ifPresent(c -> c.setRumble(RumbleType.kBothRumble, 0));

                                            } else {

                                                // controller.ifPresent( c -> c.setRumble(RumbleType.kBothRumble, 1));
                                            }

                                        }, Robot.Shooter, Robot.Spindexer)
                                )
                        )

                        .finallyDo(__ -> {

                            Robot.Shooter.stop();
                            Robot.Spindexer.stop();

                        //     controller.ifPresent(
                        //             c -> c.setRumble(
                        //                     RumbleType.kBothRumble,
                        //                     0));
                        })
        )
        .withName("AimAndShoot-" + target);
    }

    public Command shootWithoutAngleCheckCommand(
            TurretUtil.TargetType target,
            Optional<GenericHID> controller) {

        return Commands.run(() -> {

            var pose = Robot.Drivetrain.getState().Pose;

            TurretUtil.ShotSolution solution =
                    TurretUtil.computeShotSolution(pose, target);

            Robot.Spindexer.clockwise(Constants.Spindexer.kIndexingSpeed);
            Robot.Shooter.setVelocity(solution.shooterSpeedRPS);

        //     controller.ifPresent(
        //             c -> c.setRumble(RumbleType.kBothRumble, 0));

        }, Robot.Shooter, Robot.Spindexer)

        .finallyDo(__ -> {

            Robot.Shooter.stop();
            Robot.Spindexer.stop();

        //     controller.ifPresent(
                //     c -> c.setRumble(RumbleType.kBothRumble, 0));
        })
        .withName("ShootWithoutAngleCheck-" + target);
    }

    public Command shootAtSpeedWithoutAngleCheckCommand(
            double rps,
            Optional<GenericHID> controller) {

        return Commands.run(() -> {

            Robot.Spindexer.clockwise(Constants.Spindexer.kIndexingSpeed);
            Robot.Shooter.setVelocity(rps);

        }, Robot.Shooter, Robot.Spindexer)

        .finallyDo(__ -> {

            Robot.Shooter.stop();
            Robot.Spindexer.stop();

        //     controller.ifPresent(
                //     c -> c.setRumble(RumbleType.kBothRumble, 0));
        });
    }

    public Command shootAtHubCommand(Optional<GenericHID> controller) {

        Supplier<Translation2d> hub =
                () -> FieldConstants.getAllianceHub().toTranslation2d();

        return Commands.sequence(

                rotateIntoRangeCommand(
                        TurretUtil.TargetType.AllianceHUB,
                        hub).withTimeout(4),

                aimAndShootCommand(
                        TurretUtil.TargetType.AllianceHUB,
                        controller))

                .withName("ShootAtHub");
    }

    public Command autoShootAtHubCommand(Optional<GenericHID> controller) {

        Supplier<Translation2d> hub =
                () -> FieldConstants.getAllianceHub().toTranslation2d();

        return Commands.sequence(

                rotateIntoRangeCommand(
                        TurretUtil.TargetType.AllianceHUB,
                        hub),

                Commands.parallel(

                        Robot.Turret.autoAimCommand(
                                () -> Robot.Drivetrain.getState().Pose,
                                () -> TurretUtil.TargetType.AllianceHUB,
                                () -> Robot.Drivetrain.getState().Speeds,
                                () -> Units.degreesToRadians(Robot.Drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble()),
                                controller),

                        Commands.sequence(

                                Commands.waitSeconds(2.0),

                                Commands.waitUntil(() ->
                                        Math.abs(Robot.Turret.getErrorDegrees())
                                                < kTurretAlignToleranceDeg),

                                Commands.run(() -> {

                                    var pose = Robot.Drivetrain.getState().Pose;

                                    TurretUtil.ShotSolution solution =
                                            TurretUtil.computeShotSolution(
                                                    pose,
                                                    TurretUtil.TargetType.AllianceHUB);

                                    if (solution.isValid) {

                                        Robot.Spindexer.clockwise(Constants.Spindexer.kIndexingSpeed);
                                        Robot.Shooter.setVelocity(
                                                solution.shooterSpeedRPS);
                                    }

                                }, Robot.Shooter, Robot.Spindexer)

                                .finallyDo(__ -> {

                                    Robot.Shooter.stop();
                                    Robot.Spindexer.stop();
                                })
                        )
                )
        )
        .withName("AutoShootAtHub");
    }

    public Command passLeftCommand(Optional<GenericHID> controller) {

        Supplier<Translation2d> target =
                () -> FieldConstants.leftPassTarget
                        .getTranslation()
                        .toTranslation2d();

        return Commands.sequence(

                rotateIntoRangeCommand(
                        TurretUtil.TargetType.LEFT_PASS,
                        target),

                aimAndShootCommand(
                        TurretUtil.TargetType.LEFT_PASS,
                        controller))

                .withName("PassLeft");
    }

    public Command passRightCommand(Optional<GenericHID> controller) {

        Supplier<Translation2d> target =
                () -> FieldConstants.rightPassTarget
                        .getTranslation()
                        .toTranslation2d();

        return Commands.sequence(

                rotateIntoRangeCommand(
                        TurretUtil.TargetType.RIGHT_PASS,
                        target),

                aimAndShootCommand(
                        TurretUtil.TargetType.RIGHT_PASS,
                        controller))

                .withName("PassRight");
    }
}