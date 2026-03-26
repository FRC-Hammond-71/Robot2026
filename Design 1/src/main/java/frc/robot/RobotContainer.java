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
import frc.robot.telemetry.Telemetry;

public class RobotContainer {

	public Robot Robot;

	private static final Pose2d kLeftStart = new Pose2d(3.55, 6, Rotation2d.kZero);
	private static final Pose2d kMiddleStart = new Pose2d(3.55, 4, Rotation2d.kZero);
	private static final Pose2d kRightStart = new Pose2d(3.55, 2, Rotation2d.kZero);

	private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
			.withDeadband(Constants.Drivetrain.kCruiseSpeed * 0.1)
			.withRotationalDeadband(Constants.Drivetrain.kCruiseAngularRate.in(RadiansPerSecond) * 0.1)
			.withDriveRequestType(DriveRequestType.Velocity);

	private final SwerveRequest.FieldCentricFacingAngle driveHeadingKeep =
			new SwerveRequest.FieldCentricFacingAngle()
					.withDeadband(Constants.Drivetrain.kCruiseSpeed * 0.1)
					.withDriveRequestType(DriveRequestType.Velocity);

	private final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();
	private final SwerveRequest.SwerveDriveBrake xLockRequest = new SwerveRequest.SwerveDriveBrake();

	private Rotation2d headingKeepTarget = null;
	private boolean wasRotating = false;

	private final Telemetry logger = new Telemetry(Constants.Drivetrain.kCruiseSpeed);

	private final SendableChooser<Command> autoChooser;
	private final SendableChooser<Pose2d> startingPoseChooser = new SendableChooser<>();

	private final GameCommands gameCommands;

	private final Optional<GenericHID> driverController = Optional.of(Controllers.Joystick.getHID());

	private SlewRateLimiter xLimiter = new SlewRateLimiter(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 2);
	private SlewRateLimiter yLimiter = new SlewRateLimiter(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 2);
	private SlewRateLimiter rotLimiter = new SlewRateLimiter(Math.PI * 4);


	public RobotContainer(Robot robot) {

		this.Robot = robot;

		driveHeadingKeep.HeadingController.setPID(7, 0, 0.3);
		driveHeadingKeep.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

		gameCommands = new GameCommands(Robot);

		gameCommands.registerNamedCommands();
		registerNamedCommands();

		autoChooser = AutoBuilder.buildAutoChooser();
		autoChooser.addOption("Wheel Radius Characterization", buildWheelRadiusCalibrationCommand());
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

		// Alliance override — use when DS doesn't report alliance (no FMS)
		SmartDashboard.putBoolean("ForceRedAlliance", FieldConstants.forceRedAlliance);
	}

	private void registerNamedCommands() {

		NamedCommands.registerCommand("ExtendIntakeCommand",
				Commands.defer(() -> Robot.Intake.extendCommand(0.5), Set.of()));
		NamedCommands.registerCommand("RetractIntakeCommand",
				Commands.defer(() -> Robot.Intake.retractCommand(0.5), Set.of()));

		NamedCommands.registerCommand("Shoot",
				Commands.defer(() -> new ShootFuelCommand(Robot.Shooter, Robot.Drivetrain), Set.of()));

		NamedCommands.registerCommand("AimHub",
				Commands.defer(() -> Robot.Turret.autoAimCommand(() -> Robot.Drivetrain.getState().Pose, () -> TargetType.AllianceHUB, () -> Robot.Drivetrain.getState().Speeds, () -> Units.degreesToRadians(Robot.Drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble()), Optional.empty()), Set.of()));

		// NamedCommands.registerCommand("AimLeftPass",
		// 		Robot.Turret.autoAimCommand(() -> Robot.Drivetrain.getState().Pose, () -> TargetType.LEFT_PASS));

		// NamedCommands.registerCommand("AimRightPass",
		// 		Robot.Turret.autoAimCommand(() -> Robot.Drivetrain.getState().Pose, () -> TargetType.RIGHT_PASS));

		NamedCommands.registerCommand("StopTurret",
				Commands.defer(() -> Robot.Turret.stopCommand(), Set.of()));
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
				double rotVel = rotLimiter.calculate(rotInput * Constants.Drivetrain.kCruiseAngularRate.in(RadiansPerSecond) * speedScale);

				double velX = xLimiter.calculate(-Controllers.Joystick.getLeftY() * currentSpeed);
				double velY = yLimiter.calculate(-Controllers.Joystick.getLeftX() * currentSpeed);

				boolean isTranslating = Math.abs(velX) > 0.01 || Math.abs(velY) > 0.01;
				boolean isRotating = rotVel != 0;

				// No input at all — coast idle (no steer correction, no X-lock)
				if (!isTranslating && !isRotating) {
					wasRotating = false;
					headingKeepTarget = null;
					return idleRequest;
				}

				if (isRotating) {
					wasRotating = true;
					headingKeepTarget = null;
					return drive
						.withVelocityX(velX)
						.withVelocityY(velY)
						.withRotationalRate(rotVel);
				}

				// Transition from rotating to not rotating — capture heading with gyro latency compensation
				if (wasRotating || headingKeepTarget == null) {
					double yawLatencySeconds = Robot.Drivetrain.getPigeon2().getYaw().getTimestamp().getLatency();
					double angularVelocityDegPerSec = Robot.Drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble();
					Rotation2d currentYaw = Robot.Drivetrain.getState().Pose.getRotation();
					Rotation2d fieldAbsoluteHeading = currentYaw.plus(Rotation2d.fromDegrees(angularVelocityDegPerSec * yawLatencySeconds));
					// Transform from field-absolute to operator-relative frame
					// CTRE's FieldCentricFacingAngle with OperatorPerspective adds operatorForwardDirection to TargetDirection,
					// so we subtract it here to cancel that out
					headingKeepTarget = fieldAbsoluteHeading.minus(Robot.Drivetrain.getOperatorForwardDirection());
					wasRotating = false;
				}

				return driveHeadingKeep
						.withVelocityX(velX)
						.withVelocityY(velY)
						.withTargetDirection(headingKeepTarget);

			}));

		Controllers.Operator.pov(270).onTrue(Commands.runOnce(() ->
				Robot.Turret.adjustOperatorOffsetDegrees(-0.1), Robot.Turret));

		Controllers.Operator.pov(90).onTrue(Commands.runOnce(() ->
				Robot.Turret.adjustOperatorOffsetDegrees(0.1), Robot.Turret));

		RobotModeTriggers.disabled()
				.whileTrue(Robot.Drivetrain.applyRequest(() -> idleRequest).ignoringDisable(true));

		RobotModeTriggers.teleop().onTrue(Commands.runOnce(() -> {
			headingKeepTarget = null;
			wasRotating = false;
		}));

		Robot.Turret.setDefaultCommand(
				Robot.Turret.autoAimCommand(
						() -> Robot.Drivetrain.getState().Pose,
						() -> TargetType.ClosestHUB,
						() -> Robot.Drivetrain.getState().Speeds,
						() -> Units.degreesToRadians(Robot.Drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble()),
						driverController,
						false));

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
				.whileTrue(gameCommands.passClosestCommand(driverController));

		RobotModeTriggers.teleop().and(Controllers.Operator.y()).onTrue(Robot.Spindexer.counterClockwiseCommand(Constants.Spindexer.kIndexingSpeed));

		RobotModeTriggers.teleop()
				.and(Controllers.Joystick.leftBumper())
				.whileTrue(Robot.Turret.neutralOutputCommand());

		Controllers.Joystick.rightBumper()
				.whileTrue(Robot.Drivetrain.applyRequest(() -> xLockRequest));

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
					Robot.Turret.getActiveTargetType());

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

		RobotModeTriggers.test().onTrue(Robot.Turret.neutralOutputCommand()); // Don't auto aim.
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

	private Command buildWheelRadiusCalibrationCommand() {
		// driveBaseRadius = distance from robot center to any module
		// Module positions are at (±10.375", ±10.375") from center
		final double driveBaseRadiusMeters = Units.inchesToMeters(
				Math.hypot(10.375, 10.375));
		final double maxAngularVelocity = 0.25; // rad/s — slow spin
		final double rampRate = 0.05; // rad/s² — smooth acceleration

		final SlewRateLimiter limiter = new SlewRateLimiter(rampRate);
		final double driveGearRatio = 8.142857142857142; // MK4i L1

		SwerveRequest.RobotCentric spin = new SwerveRequest.RobotCentric()
				.withDriveRequestType(DriveRequestType.Velocity);

		// State variables captured by lambda
		final double[] startPositions = new double[4];
		final double[] lastYaw = {0};
		final double[] gyroDelta = {0};

		return Commands.parallel(
			// Drive: ramp up angular velocity
			Commands.sequence(
				Commands.runOnce(() -> limiter.reset(0.0)),
				Robot.Drivetrain.applyRequest(() ->
					spin.withRotationalRate(limiter.calculate(maxAngularVelocity)))
			),
			// Measure: wait for modules to orient, then track gyro vs wheels
			Commands.sequence(
				Commands.waitSeconds(1.0), // let modules orient
				Commands.runOnce(() -> {
					// Record starting positions
					for (int i = 0; i < 4; i++) {
						startPositions[i] = Robot.Drivetrain.getModule(i)
								.getDriveMotor().getPosition().getValueAsDouble()
								* 2.0 * Math.PI / driveGearRatio; // motor rotations → wheel radians
					}
					lastYaw[0] = Robot.Drivetrain.getPigeon2().getYaw().getValueAsDouble();
					gyroDelta[0] = 0.0;
					SmartDashboard.putString("WheelRadiusCharacterization/Status", "Measuring...");
				}),
				Commands.run(() -> {
					// Accumulate absolute gyro rotation
					double currentYaw = Robot.Drivetrain.getPigeon2().getYaw().getValueAsDouble();
					double yawDeltaDeg = currentYaw - lastYaw[0];
					// Handle wrap-around
					if (yawDeltaDeg > 180) yawDeltaDeg -= 360;
					if (yawDeltaDeg < -180) yawDeltaDeg += 360;
					gyroDelta[0] += Math.abs(Math.toRadians(yawDeltaDeg));
					lastYaw[0] = currentYaw;

					// Calculate average wheel delta across all 4 modules
					double wheelDelta = 0.0;
					for (int i = 0; i < 4; i++) {
						double currentPos = Robot.Drivetrain.getModule(i)
								.getDriveMotor().getPosition().getValueAsDouble()
								* 2.0 * Math.PI / driveGearRatio;
						wheelDelta += Math.abs(currentPos - startPositions[i]) / 4.0;
					}

					// Calculate wheel radius
					double wheelRadiusMeters = wheelDelta > 0.01
							? (gyroDelta[0] * driveBaseRadiusMeters) / wheelDelta
							: 0.0;
					double wheelRadiusInches = Units.metersToInches(wheelRadiusMeters);

					SmartDashboard.putNumber("WheelRadiusCharacterization/GyroDeltaRad", gyroDelta[0]);
					SmartDashboard.putNumber("WheelRadiusCharacterization/WheelDeltaRad", wheelDelta);
					SmartDashboard.putNumber("WheelRadiusCharacterization/WheelRadiusInches", wheelRadiusInches);
					SmartDashboard.putNumber("WheelRadiusCharacterization/WheelRadiusMeters", wheelRadiusMeters);
				})
			)
		).finallyDo(() -> {
			double wheelDelta = 0.0;
			for (int i = 0; i < 4; i++) {
				double currentPos = Robot.Drivetrain.getModule(i)
						.getDriveMotor().getPosition().getValueAsDouble()
						* 2.0 * Math.PI / driveGearRatio;
				wheelDelta += Math.abs(currentPos - startPositions[i]) / 4.0;
			}
			double wheelRadiusMeters = wheelDelta > 0.01
					? (gyroDelta[0] * driveBaseRadiusMeters) / wheelDelta
					: 0.0;
			double wheelRadiusInches = Units.metersToInches(wheelRadiusMeters);

			System.out.println("********** Wheel Radius Characterization Results **********");
			System.out.printf("    Gyro Delta:   %.6f radians%n", gyroDelta[0]);
			System.out.printf("    Wheel Delta:  %.6f radians%n", wheelDelta);
			System.out.printf("    Wheel Radius: %.6f meters (%.5f inches)%n",
					wheelRadiusMeters, wheelRadiusInches);
			System.out.println("    Update TunerConstants.kWheelRadius = Inches.of(" +
					String.format("%.5f", wheelRadiusInches) + ");");

			SmartDashboard.putString("WheelRadiusCharacterization/Status",
					String.format("DONE - Radius: %.5f inches. Update TunerConstants.", wheelRadiusInches));
		}).withName("Wheel Radius Characterization");
	}

	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}
}
