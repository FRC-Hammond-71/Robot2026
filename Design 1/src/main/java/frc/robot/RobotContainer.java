// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.Set;

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
import frc.robot.Commands.GameCommands;
import frc.robot.Commands.ShooterTuningCommand;
import frc.robot.Limelight.LimelightHelpers;
import frc.robot.Commands.ShootFuelCommand;

public class RobotContainer {

	Optional<Alliance> ally = DriverStation.getAlliance();
	public double x;
	public double y;
	public double z;
	private final Intake intake = new Intake();
	private final Shooter shooter = new Shooter();
	private final Climber climber = new Climber();
	private final Spindexer spindexer = new Spindexer();
	private final Turret turret = new Turret();

	/*
	 * Setting up bindings for neces]\[
	 * sary control of the swerve drive platform
	 */
	private static final Pose2d kLeftStart = new Pose2d(2, 6, Rotation2d.kZero);
	private static final Pose2d kMiddleStart = new Pose2d(2, 4, Rotation2d.kZero);
	private static final Pose2d kRightStart = new Pose2d(2, 2, Rotation2d.kZero);
	private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
			.withDeadband(Constants.Drivetrain.kCruiseSpeed * 0.1)
			.withRotationalDeadband(Constants.Drivetrain.kCruiseAngularRate.in(RadiansPerSecond) * 0.1) // Add a 10%
																										// deadband
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
	private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
	private final SwerveRequest.FieldCentricFacingAngle driveLookingAtHub = new SwerveRequest.FieldCentricFacingAngle()
			.withDeadband(Constants.Drivetrain.kCruiseSpeed * 0.1)
			.withRotationalDeadband(Constants.Drivetrain.kCruiseAngularRate.in(RadiansPerSecond) * 0.1)
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage);
	private final SwerveRequest.FieldCentricFacingAngle headingKeep = new SwerveRequest.FieldCentricFacingAngle()
			.withDeadband(Constants.Drivetrain.kCruiseSpeed * 0.1)
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage);
	private Rotation2d lastHeading = Rotation2d.kZero;
	private boolean wasRotating = true; // start true so first loop captures actual heading
	private final Telemetry logger = new Telemetry(Constants.Drivetrain.kCruiseSpeed);
	private final CommandXboxController joystick = new CommandXboxController(0);
	public final CommandSwerveDrivetrain drivetrain = new CommandSwerveDrivetrain(
			turret.getPositionSignal(),
			TunerConstants.DrivetrainConstants,
			TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight);
	// private final SendableChooser<Command> autoChooser;
	private final SendableChooser<Pose2d> startingPoseChooser = new SendableChooser<>();
	private final GameCommands gameCommands;

	// #endregion
	// #region Commands

	public RobotContainer() {
		gameCommands = new GameCommands(shooter, turret, drivetrain, spindexer, joystick);
		gameCommands.registerNamedCommands(); // Registers ShootAtHub, PassLeft, PassRight
		registerNamedCommands(); // Registers fine-grained subsystem named commands
		// autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
		// SmartDashboard.putData("Auto Mode", autoChooser);
		configureBindings();

		startingPoseChooser.setDefaultOption("Middle", kMiddleStart);
		startingPoseChooser.addOption("Left", kLeftStart);
		startingPoseChooser.addOption("Right", kRightStart);
		startingPoseChooser.onChange(pose -> drivetrain.resetPose(getStartingPose()));
		SmartDashboard.putData("Starting Position", startingPoseChooser);
	}

	private void registerNamedCommands() {
		// Intake
		NamedCommands.registerCommand("Intake", intake.intakeCommand(1.0));
		NamedCommands.registerCommand("Score", intake.scoreCommand(1.0));

		// Spindexer
		NamedCommands.registerCommand("SpindexerIn", spindexer.clockwiseCommand(1));
		NamedCommands.registerCommand("SpindexerOut", spindexer.counterClockwiseCommand(1));

		// Shooter
		NamedCommands.registerCommand("Shoot", new ShootFuelCommand(shooter, drivetrain));

		// Turret
		NamedCommands.registerCommand("AimHub",
				turret.autoAimCommand(() -> drivetrain.getState().Pose, TargetType.HUB));
		NamedCommands.registerCommand("AimLeftPass",
				turret.autoAimCommand(() -> drivetrain.getState().Pose, TargetType.LEFT_PASS));
		NamedCommands.registerCommand("AimRightPass",
				turret.autoAimCommand(() -> drivetrain.getState().Pose, TargetType.RIGHT_PASS));
		NamedCommands.registerCommand("StopTurret", turret.stopCommand());

		// Climber
		NamedCommands.registerCommand("ExtendClimber", climber.extendCommand());
		NamedCommands.registerCommand("Climb", climber.climbCommand());
	}

	private SlewRateLimiter xLimiter = new SlewRateLimiter(10);
	private SlewRateLimiter yLimiter = new SlewRateLimiter(10);
	// private SlewRateLimiter rotLimiter = new SlewRateLimiter(Math.PI);

	private void configureBindings() {

		driveLookingAtHub.HeadingController.setPID(8, 0, 0);
		driveLookingAtHub.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

		headingKeep.HeadingController.setPID(8, 0, 0);
		headingKeep.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

		// Note that X is defined as forward according to WPILib convention,
		// and Y is defined as to the left according to WPILib convention.
		drivetrain.setDefaultCommand(
				drivetrain.applyRequest(() -> {
					// Reduce speed to 50% in test mode for safety
					double speedScale = DriverStation.isTest() ? 0.5 : 1.0;
					// Boost: hold left trigger to unlock full drivetrain speed
					double currentSpeed = joystick.getLeftTriggerAxis() > 0.2
							? TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)
							: Constants.Drivetrain.kCruiseSpeed;

					currentSpeed *= speedScale;

					double rotInput = -joystick.getRightX();
					boolean isRotating = Math.abs(rotInput) > 0.1;

					if (isRotating) {
						wasRotating = true;
						return drive
								.withVelocityX(xLimiter.calculate(-joystick.getLeftY() * currentSpeed))
								.withVelocityY(yLimiter.calculate(-joystick.getLeftX() * currentSpeed))
								.withRotationalRate(rotInput * Constants.Drivetrain.kCruiseAngularRate.in(RadiansPerSecond) * speedScale);
					} else {
						// Capture heading when driver stops rotating
						if (wasRotating) {
							lastHeading = drivetrain.getState().Pose.getRotation();
							wasRotating = false;
						}
						return headingKeep
								.withVelocityX(xLimiter.calculate(-joystick.getLeftY() * currentSpeed))
								.withVelocityY(yLimiter.calculate(-joystick.getLeftX() * currentSpeed))
								.withTargetDirection(lastHeading);
					}

				}));

		// Idle while the robot is disabled. This ensures the configured
		// neutral mode is applied to the drive motors while disabled.
		final var idle = new SwerveRequest.Idle();
		RobotModeTriggers.disabled().whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

		// Default turret behavior: always track the hub in teleop
		turret.setDefaultCommand(
				turret.autoAimCommand(
						() -> drivetrain.getState().Pose,
						TargetType.HUB,
						() -> drivetrain.getState().Speeds.omegaRadiansPerSecond));

		RobotModeTriggers.teleop().and(joystick.a()).whileTrue(gameCommands.aimAndShootCommand(TargetType.HUB));

		// B: pass to whichever side of the field the robot is currently on.
		// Y > 4.0 m = upper half → left pass target (Y=6.0); lower half → right pass
		// target (Y=1.96).
		RobotModeTriggers.teleop().and(joystick.b()).whileTrue(
				Commands.defer(
						() -> drivetrain.getState().Pose.getY() > 4.0
								? gameCommands.passLeftCommand()
								: gameCommands.passRightCommand(),
						Set.of(turret, shooter, spindexer, drivetrain)));

		joystick.start().onTrue(drivetrain.runOnce(() -> {
			drivetrain.seedFieldCentric();
			lastHeading = Rotation2d.kZero;
			wasRotating = true;
		}));

		joystick.rightBumper().onTrue(intake.toggleExtensionCommand());

		// configureTestBindingsForShooterTuning();
		configureTestBindingsForManualShootingAndTurret();

		drivetrain.registerTelemetry(logger::telemeterize);
	}

	private void configureTestBindingsForManualShootingAndTurret() {
		// A: turret auto-tracking (no shooter)
		RobotModeTriggers.test().and(joystick.a()).whileTrue(
				turret.autoAimCommand(
						() -> drivetrain.getState().Pose,
						TargetType.HUB,
						() -> drivetrain.getState().Speeds.omegaRadiansPerSecond));

		// Left trigger: manual shooter + spindexer control
		RobotModeTriggers.test().whileTrue(Commands.runEnd(() -> {

			if (joystick.getLeftTriggerAxis() > 0.2) {
				spindexer.clockwise(0.5);
			} else {
				spindexer.stop();
			}
			shooter.setVelocity(joystick.getLeftTriggerAxis() * Constants.Shooter.kMaxSpeedRPS);

		}, () -> {
			shooter.stop();
			spindexer.stop();
		}));

		// B: turret aim + shooter spinup (full shot readiness, no feeder)
		// RobotModeTriggers.test().and(joystick.b()).whileTrue(
		// gameCommands.aimAndShootCommand(TargetType.HUB));

		// X: rotate robot so the hub falls inside the turret's allowed range
		RobotModeTriggers.test().and(joystick.x()).whileTrue(
				gameCommands.rotateIntoRangeCommand(
						TargetType.HUB,
						() -> FieldConstants.getAllianceHub().toTranslation2d()));

		// POV: manual turret angle control
		RobotModeTriggers.test().and(joystick.pov(270)).whileTrue(Commands.run(() -> turret.setAngle(90), turret));
		RobotModeTriggers.test().and(joystick.pov(180)).whileTrue(Commands.run(() -> turret.setAngle(180), turret));
		RobotModeTriggers.test().and(joystick.pov(90)).whileTrue(Commands.run(() -> turret.setAngle(270), turret));

		// Y: reset turret encoder
		// joystick.y().onTrue(Commands.runOnce(() -> turret.resetAngle(), drivetrain));

		// Intake — manual extension override for testing current threshold
		RobotModeTriggers.test().and(joystick.rightTrigger()).whileTrue(intake.extendCommand(0.30));
		RobotModeTriggers.test().and(joystick.rightBumper()).whileTrue(intake.retractCommand(0.30));
		joystick.leftBumper().whileTrue(intake.intakeCommand(1.0));

		// Climber manual overrides
		// joystick.back().and(joystick.pov(0)).whileTrue(climber.upCommand(1));
		// joystick.back().and(joystick.pov(180)).whileTrue(climber.downCommand(1));
	}

	private void configureTestBindingsForShooterTuning() {
		joystick.b().onTrue(new ShooterTuningCommand(
				shooter,
				spindexer,
				RotationsPerSecond.of(Constants.Shooter.kMinSpeedRPS),
				RotationsPerSecond.of(Constants.Shooter.kMaxSpeedRPS),
				RotationsPerSecond.of(2.5),
				() -> joystick.y().getAsBoolean(),
				() -> joystick.a().getAsBoolean(),
				() -> joystick.x().getAsBoolean()));
	}

	// dear claude, fix thiss code, 6 7

	// public Command getAutonomousCommand() {p[]p[][]\

	// // Simple drive forward auton
	// final var idle = new SwerveRequest.Idle();
	// return Commands.sequence(
	// // Reset our field centric heading to match the robot
	// // facing away from our alliance station wall (0 deg).
	// drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
	// // Then slowly drive forward (away from us) for 5 seconds.
	// drivetrain.applyRequest(() ->
	// drive.withVelocityX(0.5)
	// .withVelocityY(0)
	// .withRotationalRate(0)
	// )
	// .withTimeout(5.0),
	// // Finally idle for the rest of auton
	// drivetrain.applyRequest(() -> idle)
	// );

	// return autoChooser.getSelected();
	// }

	public Pose2d getStartingPose() {
		Pose2d pose = startingPoseChooser.getSelected();
		if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
			return new Pose2d(16.54 - pose.getX(), pose.getY(), pose.getRotation().plus(Rotation2d.k180deg));
		}
		return pose;
	}

	public Command getAutonomousCommand() {
		return gameCommands.autoShootAtHubCommand();
	}

}
