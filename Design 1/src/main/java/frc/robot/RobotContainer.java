// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
import frc.robot.generated.FieldConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.subsystems.Turret;
import frc.robot.util.dashboard.TurretUtil;
import frc.robot.util.dashboard.TurretUtil.TargetType;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Commands.GameCommands;
import frc.robot.Commands.ShootFuelCommand;

public class RobotContainer {
    
    //#region Initialization
    private double MaxSpeed = 1 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1.5).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    Optional<Alliance> ally = DriverStation.getAlliance();
    public double x;
    public double y; 
    public double z;
    private final Intake intake = new Intake();
    private final Shooter shooter = new Shooter();
    private final Climber climber = new Climber();
    private final Spindexer spindexer = new Spindexer();
    private final Turret turret = new Turret();
    
    /* Setting up bindings for neces]\[
     sary control of the swerve drive platform */
    private static final Pose2d kStartingPose = new Pose2d(8.774, 4.026, Rotation2d.fromDegrees(0));
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.FieldCentricFacingAngle driveLookingAtHub = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final CommandXboxController joystick = new CommandXboxController(0);
    public final CommandSwerveDrivetrain drivetrain = new CommandSwerveDrivetrain(
        turret.getPositionSignal(),
        TunerConstants.DrivetrainConstants,
        TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight
    );
    private final SendableChooser<Command> autoChooser;
    private final GameCommands gameCommands;


    //#endregion
    //#region Commands


    public RobotContainer() {
        gameCommands = new GameCommands(shooter, turret, drivetrain);
        gameCommands.registerNamedCommands(); // Registers ShootAtHub, PassLeft, PassRight
        registerNamedCommands(); // Registers fine-grained subsystem named commands
        autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
        configureBindings();
        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    private void registerNamedCommands() {
        // Intake
        NamedCommands.registerCommand("Intake",      intake.intakeCommand(1.0));
        NamedCommands.registerCommand("Score",       intake.scoreCommand(1.0));

        // Spindexer
        NamedCommands.registerCommand("SpindexerIn",  spindexer.clockwiseCommand(1));
        NamedCommands.registerCommand("SpindexerOut", spindexer.counterClockwiseCommand(1));

        // Shooter
        NamedCommands.registerCommand("Shoot", new ShootFuelCommand(shooter, drivetrain));

        // Turret
        NamedCommands.registerCommand("AimHub",       turret.autoAimCommand(() -> drivetrain.getState().Pose, drivetrain::getFieldRelativeSpeeds, TargetType.HUB));
        NamedCommands.registerCommand("AimLeftPass",  turret.autoAimCommand(() -> drivetrain.getState().Pose, drivetrain::getFieldRelativeSpeeds, TargetType.LEFT_PASS));
        NamedCommands.registerCommand("AimRightPass", turret.autoAimCommand(() -> drivetrain.getState().Pose, drivetrain::getFieldRelativeSpeeds, TargetType.RIGHT_PASS));
        NamedCommands.registerCommand("TurretForward", turret.moveToAngleCommand(0));
        NamedCommands.registerCommand("StopTurret",    turret.stopCommand());

        // Climber
        NamedCommands.registerCommand("ExtendClimber", climber.extendCommand());
        NamedCommands.registerCommand("Climb",         climber.climbCommand());
    }

    private SlewRateLimiter xLimiter = new SlewRateLimiter(2);
    private SlewRateLimiter yLimiter = new SlewRateLimiter(2);
    private SlewRateLimiter rotLimiter = new SlewRateLimiter(Math.PI);

    private void configureBindings() {

        driveLookingAtHub.HeadingController.setPID(5, 0, 0);
        driveLookingAtHub.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                // Reduce speed to 30% in test mode for safety
                double speedScale = DriverStation.isTest() ? 0.3 : 1.0;

                // B: face hub — teleop only (B is used for aim+shoot in test mode)
                if (!DriverStation.isTest() && joystick.b().getAsBoolean()) {
                    Pose2d robotPose = drivetrain.getState().Pose;
                    Translation3d hub = FieldConstants.getAllianceHub();
                    Rotation2d angleToHub = new Rotation2d(
                        hub.getX() - robotPose.getX(),
                        hub.getY() - robotPose.getY()
                    );
                    return driveLookingAtHub
                        .withVelocityX(-joystick.getLeftY() * MaxSpeed)
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                        .withTargetDirection(angleToHub);
                }

                return drive
                    .withVelocityX(-joystick.getLeftY() * MaxSpeed * speedScale)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed * speedScale)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate * speedScale);
            })
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));
        
        // turret.autoAimCommand(() -> drivetrain.getState().Pose, drivetrain::getFieldRelativeSpeeds, TargetType.HUB);
        
        // Turret continuously auto-aims at the hub during teleop (and test when idle).
        turret.setDefaultCommand(
            turret.autoAimCommand(
                () -> drivetrain.getState().Pose,
                drivetrain::getFieldRelativeSpeeds,
                TargetType.HUB
            )
        );

        // All test commands run only while button is held; release = immediate stop.
        // Drive is reduced to 30% speed automatically (see default command above).

        // A: turret auto-tracking with lead compensation (no shooter)
        RobotModeTriggers.test().and(joystick.a()).whileTrue(
            turret.autoAimCommand(
                () -> drivetrain.getState().Pose,
                drivetrain::getFieldRelativeSpeeds,
                TargetType.HUB
            )
        );

        RobotModeTriggers.test().whileTrue(Commands.runEnd(() -> {

            shooter.setVelocity(joystick.getLeftTriggerAxis()  * ShooterConstants.kMaxSpeedRPS);

        }, () -> shooter.stop()));

        // B: turret aim + shooter spinup (full shot readiness, no feeder)
        RobotModeTriggers.test().and(joystick.b()).whileTrue(
            gameCommands.aimAndShootCommand(TargetType.HUB)
        );

        // X: rotate robot so the hub falls inside the turret's allowed range
        RobotModeTriggers.test().and(joystick.x()).whileTrue(
            gameCommands.rotateIntoRangeCommand(
                TargetType.HUB,
                () -> FieldConstants.getAllianceHub().toTranslation2d()
            )
        );

        joystick.rightBumper().whileTrue(intake.retractCommand(0.30));
        joystick.rightTrigger().whileTrue(intake.extendCommand(0.30));

        joystick.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // Left bumper: intake (hold to run, release to stop)
        joystick.leftBumper().whileTrue(intake.intakeCommand(-5.0));

        

        // Full climb sequence: align (odometry) → extend → climb
        // joystick.start().onTrue(
        //     Commands.sequence(
        //         AutoBuilder.pathfindToPose(Constants.Climber.kClimbAlignPose, Constants.Climber.kAlignPathConstraints),
        //         climber.extendCommand(),
        //         climber.climbCommand()
        //     ).withName("Full Climb Sequence")
        // );

        // Manual overrides (hold back + POV for fine control)
        joystick.back().and(joystick.pov(0)).whileTrue(climber.upCommand(1));
        joystick.back().and(joystick.pov(180)).whileTrue(climber.downCommand(1));

        joystick.pov(90)
        .whileTrue(spindexer.clockwiseCommand(1));
        joystick.pov(270)
        .whileTrue(spindexer.counterClockwiseCommand(1));

        joystick.y().onTrue(
            Commands.runOnce(() -> {
                // drivetrain.resetPose(kStartingPose);
                turret.resetAngle();
            }, drivetrain)
        );

        // // Run SysId routines when holding back/start and X/Y.
        // // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        drivetrain.registerTelemetry(logger::telemeterize);
        SmartDashboard.putData("Auto Selection", autoChooser);
    }

    // public Command getAutonomousCommand() {p[]p[][]\

        // // Simple drive forward auton
        // final var idle = new SwerveRequest.Idle();
        // return Commands.sequence(
        //     // Reset our field centric heading to match the robot
        //     // facing away from our alliance station wall (0 deg).
        //     drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
        //     // Then slowly drive forward (away from us) for 5 seconds.
        //     drivetrain.applyRequest(() ->
        //         drive.withVelocityX(0.5)
        //             .withVelocityY(0)
        //             .withRotationalRate(0)
        //     )
        //     .withTimeout(5.0),
        //     // Finally idle for the rest of auton
        //     drivetrain.applyRequest(() -> idle)
        // );

        // return autoChooser.getSelected();
    // }


    
    public Command getAutonomousCommand() {
        // This method loads the auto when it is called, however, it is recommended
        // to first load your paths/autos when code starts, then return the
        // pre-loaded auto/path
        return autoChooser.getSelected();
    }
          





}
