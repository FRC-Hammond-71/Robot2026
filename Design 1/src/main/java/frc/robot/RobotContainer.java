package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.Set;

import org.ironmaple.simulation.SimulatedArena;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.robot.generated.FieldConstants;
import frc.robot.generated.TunerConstants;

import frc.robot.subsystems.*;

import frc.robot.util.dashboard.TurretUtil;
import frc.robot.util.dashboard.TurretUtil.TargetType;

import frc.robot.Commands.*;

public class RobotContainer {

	public Robot Robot;

	private static final Pose2d kLeftStart = new Pose2d(3.55, 6, Rotation2d.kZero);
	private static final Pose2d kMiddleStart = new Pose2d(3.55, 4, Rotation2d.kZero);
	private static final Pose2d kRightStart = new Pose2d(3.55, 2, Rotation2d.kZero);

	private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
			.withDeadband(Constants.Drivetrain.kCruiseSpeed * 0.1)
			.withRotationalDeadband(Constants.Drivetrain.kCruiseAngularRate.in(RadiansPerSecond) * 0.1)
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage);

	private final SwerveRequest.FieldCentricFacingAngle driveHeadingKeep =
			new SwerveRequest.FieldCentricFacingAngle()
					.withDeadband(Constants.Drivetrain.kCruiseSpeed * 0.1)
					.withDriveRequestType(DriveRequestType.OpenLoopVoltage);

	private final Telemetry logger = new Telemetry(Constants.Drivetrain.kCruiseSpeed);

	private final SendableChooser<Command> autoChooser;
	private final SendableChooser<Pose2d> startingPoseChooser = new SendableChooser<>();

	private final GameCommands gameCommands;

	private final Optional<GenericHID> driverController = Optional.of(Controllers.Joystick.getHID());

	private SlewRateLimiter xLimiter = new SlewRateLimiter(Constants.Drivetrain.kCruiseSpeed * 4);
	private SlewRateLimiter yLimiter = new SlewRateLimiter(Constants.Drivetrain.kCruiseSpeed * 4);

	private TargetType currentTurretTarget = TargetType.AllianceHUB;
	private Rotation2d headingKeepTarget = null;
	private boolean wasRotating = false;

	public RobotContainer(Robot robot) {

		this.Robot = robot;

		gameCommands = new GameCommands(Robot);

		driveHeadingKeep.HeadingController.setPID(2, 0, 0.1);
		driveHeadingKeep.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

		gameCommands.registerNamedCommands();
		registerNamedCommands();

		autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("Auto Mode", autoChooser);

		autoChooser.onChange((chosenCmd) -> { Robot.selectedAutoCommand = chosenCmd; });

		configureBindings();

		startingPoseChooser.setDefaultOption("Middle", kMiddleStart);
		startingPoseChooser.addOption("Left", kLeftStart);
		startingPoseChooser.addOption("Right", kRightStart);

		startingPoseChooser.onChange(pose -> {
			Robot.Drivetrain.resetPose(getStartingPose());
			headingKeepTarget = null;
		});

		SmartDashboard.putData("Starting Position", startingPoseChooser);
	}

	private void registerNamedCommands() {

		NamedCommands.registerCommand("ExtendIntakeCommand", Robot.Intake.extendCommand(0.5));
		NamedCommands.registerCommand("RetractIntakeCommand", Robot.Intake.retractCommand(0.5));

		NamedCommands.registerCommand("Shoot", new ShootFuelCommand(Robot.Shooter, Robot.Drivetrain));

		NamedCommands.registerCommand("AimHub",
				Robot.Turret.autoAimCommand(() -> Robot.Drivetrain.getState().Pose, () -> TargetType.AllianceHUB, () -> Robot.Drivetrain.getState().Speeds, () -> Units.degreesToRadians(Robot.Drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble()), Optional.empty()));

		// NamedCommands.registerCommand("AimLeftPass",
		// 		Robot.Turret.autoAimCommand(() -> Robot.Drivetrain.getState().Pose, () -> TargetType.LEFT_PASS));

		// NamedCommands.registerCommand("AimRightPass",
		// 		Robot.Turret.autoAimCommand(() -> Robot.Drivetrain.getState().Pose, () -> TargetType.RIGHT_PASS));

		NamedCommands.registerCommand("StopTurret", Robot.Turret.stopCommand());
	}

	private void configureBindings() {

		Robot.Drivetrain.setDefaultCommand(
			Robot.Drivetrain.applyRequest(() -> {

				double speedScale = DriverStation.isTest() ? 0.5 : 1.0;

				double currentSpeed = Controllers.Joystick.getLeftTriggerAxis() > 0.2
						? TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)
						: Constants.Drivetrain.kCruiseSpeed;

				currentSpeed *= speedScale;

				double rotInput = MathUtil.applyDeadband(-Controllers.Joystick.getRightX(), 0.1);

				double velX = xLimiter.calculate(-Controllers.Joystick.getLeftY() * currentSpeed);
				double velY = yLimiter.calculate(-Controllers.Joystick.getLeftX() * currentSpeed);

				boolean isRotating = rotInput != 0;

				if (isRotating) {
					wasRotating = true;
					return drive
							.withVelocityX(velX)
							.withVelocityY(velY)
							.withRotationalRate(rotInput *
									Constants.Drivetrain.kCruiseAngularRate.in(RadiansPerSecond) * speedScale);
				} else {
					if (wasRotating || headingKeepTarget == null) {
						// Transition from rotating to not rotating — capture heading with latency compensation
						double yawLatencySeconds = Robot.Drivetrain.getPigeon2().getYaw().getTimestamp().getLatency();
						double angularVelocityDegPerSec = Robot.Drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble();
						Rotation2d currentYaw = Robot.Drivetrain.getState().Pose.getRotation();
						headingKeepTarget = currentYaw.plus(Rotation2d.fromDegrees(angularVelocityDegPerSec * yawLatencySeconds));
						wasRotating = false;
					}
					// Hold the captured heading
					return driveHeadingKeep
							.withVelocityX(velX)
							.withVelocityY(velY)
							.withTargetDirection(headingKeepTarget);
				}

			}));

		Controllers.Joystick.pov(90).onTrue(Commands.runOnce(() ->
				currentTurretTarget = TargetType.RIGHT_PASS));

		Controllers.Joystick.pov(0).onTrue(Commands.runOnce(() ->
				currentTurretTarget = TargetType.AllianceHUB));

		Controllers.Joystick.pov(270).onTrue(Commands.runOnce(() ->
				currentTurretTarget = TargetType.LEFT_PASS));

		Controllers.Operator.pov(270).onTrue(Commands.runOnce(() ->
				Robot.Turret.adjustOperatorOffsetDegrees(-0.1), Robot.Turret));

		Controllers.Operator.pov(90).onTrue(Commands.runOnce(() ->
				Robot.Turret.adjustOperatorOffsetDegrees(0.1), Robot.Turret));

		final var idle = new SwerveRequest.Idle();

		RobotModeTriggers.disabled()
				.whileTrue(Robot.Drivetrain.applyRequest(() -> idle).ignoringDisable(true));

		Robot.Turret.setDefaultCommand(
				Robot.Turret.autoAimCommand(
						() -> Robot.Drivetrain.getState().Pose,
						() -> currentTurretTarget,
						() -> Robot.Drivetrain.getState().Speeds,
						() -> Units.degreesToRadians(Robot.Drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble()),
						driverController));

		RobotModeTriggers.teleop()
				.and(Controllers.Operator.a())
				.and(Controllers.Operator.leftBumper().negate())
				.whileTrue(gameCommands.aimAndShootCommand(TargetType.AllianceHUB, driverController));

		RobotModeTriggers.teleop()
				.and(Controllers.Operator.b())
				.whileTrue(gameCommands.shootAtSpeedWithoutAngleCheckCommand(45, driverController));

		// RobotModeTriggers.teleop().and(operator.y()).onTrue(spindexer.counterClockwiseCommand(0.8));

		RobotModeTriggers.teleop()
				.and(Controllers.Operator.x())
				.whileTrue(gameCommands.shootAtSpeedWithoutAngleCheckCommand(70, driverController));

		RobotModeTriggers.teleop().and(Controllers.Operator.y()).onTrue(Robot.Spindexer.counterClockwiseCommand(Constants.Spindexer.kIndexingSpeed));

		RobotModeTriggers.teleop()
				.and(Controllers.Joystick.leftBumper())
				.whileTrue(Robot.Turret.neutralOutputCommand());

		Controllers.Joystick.start().onTrue(Robot.Drivetrain.runOnce(() -> {
				Robot.Drivetrain.seedFieldCentric();
				headingKeepTarget = null;
		}));

		Controllers.Operator.rightBumper().onTrue(Robot.Intake.toggleExtensionCommand());

		configureTestBindingsForShooterTuning();
		// configureTestBindingsForManualShootingAndTurret();

		Robot.Drivetrain.registerTelemetry(state -> {

			// logger.setTurretHeadingField(Robot.Turret.getDesiredHeadingFieldRelative(state.Pose.getRotation()));
			logger.setTurretHeadingField(Rotation2d.fromDegrees(Robot.Turret.getFieldRelativeAngleDegrees(state.Pose.getRotation())));

			var shot = TurretUtil.computeLeadingShot(
					state.Pose,
					state.Speeds,
					currentTurretTarget);

			logger.setShotSolution(shot);

			Rotation2d aimField = Rotation2d.fromDegrees(shot.robotRelativeAngleDegrees)
					.plus(state.Pose.getRotation());

			logger.setLeadShotPose(new Pose2d(
					state.Pose.getX() + shot.distanceMeters * aimField.getCos(),
					state.Pose.getY() + shot.distanceMeters * aimField.getSin(),
					aimField));

			// logger.setLeadShotPose(new Pose2d(state.Pose.getTranslation(), aimField));

			FieldConstants.publishFieldPoses();

			logger.telemeterize(state);
		});
	}

	private void configureTestBindingsForManualShootingAndTurret() {

		RobotModeTriggers.test().and(Controllers.Joystick.a()).whileTrue(
				Robot.Turret.autoAimCommand(
						() -> Robot.Drivetrain.getState().Pose,
						() -> TargetType.AllianceHUB,
						() -> Robot.Drivetrain.getState().Speeds,
						() -> Units.degreesToRadians(Robot.Drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble()),
						driverController));

		RobotModeTriggers.test().whileTrue(Commands.runEnd(() -> {

			if (Controllers.Joystick.getLeftTriggerAxis() > 0.2) {
				Robot.Spindexer.clockwise(Constants.Spindexer.kIndexingSpeed);
			} else {
				Robot.Spindexer.stop();
			}

			Robot.Shooter.setVelocity(
					Controllers.Joystick.getLeftTriggerAxis()
							* Constants.Shooter.kMaxSpeedRPS);

		}, () -> {

			Robot.Shooter.stop();
			Robot.Spindexer.stop();

		}, Robot.Shooter, Robot.Spindexer));

		RobotModeTriggers.test()
				.and(Controllers.Joystick.pov(270))
				.whileTrue(Commands.run(() ->
						Robot.Turret.setRobotRelativeAngle(90), Robot.Turret));

		RobotModeTriggers.test()
				.and(Controllers.Joystick.pov(180))
				.whileTrue(Commands.run(() ->
						Robot.Turret.setRobotRelativeAngle(180), Robot.Turret));

		RobotModeTriggers.test()
				.and(Controllers.Joystick.pov(90))
				.whileTrue(Commands.run(() ->
						Robot.Turret.setRobotRelativeAngle(270), Robot.Turret));
	}

	private void configureTestBindingsForShooterTuning() {

		RobotModeTriggers.test().and(Controllers.Joystick.b()).onTrue(new ShooterTuningCommand(Robot.Shooter,
				Robot.Spindexer,
				RotationsPerSecond.of(Constants.Shooter.kMinSpeedRPS),
				RotationsPerSecond.of(Constants.Shooter.kMaxSpeedRPS),
				RotationsPerSecond.of(1),
				() -> Controllers.Joystick.y().getAsBoolean(),
				() -> Controllers.Joystick.a().getAsBoolean(),
				() -> Controllers.Joystick.x().getAsBoolean()));
	}

	public Pose2d getStartingPose() {

		Pose2d pose = startingPoseChooser.getSelected();

		if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {

			if (pose.equals(kLeftStart)) {
				return new Pose2d(16.54 - pose.getX(), pose.getY() - 4,
						pose.getRotation().plus(Rotation2d.k180deg));
			} else if (pose.equals(kRightStart)) {
				return new Pose2d(16.54 - pose.getX(), pose.getY() + 4,
						pose.getRotation().plus(Rotation2d.k180deg));
			} else {
				return new Pose2d(16.54 - pose.getX(), pose.getY(),
						pose.getRotation().plus(Rotation2d.k180deg));
			}
		}

		return pose;
	}

	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}
}