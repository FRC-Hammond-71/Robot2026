package frc.robot.Commands;

import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.dashboard.TurretUtil;

public class GameCommands {

	private static final double kTurretAlignToleranceDeg = 4;

	private final Robot Robot;

	public GameCommands(Robot robot) {

		this.Robot = robot;
	}

	/** Spins up the shooter and only feeds the spindexer once at speed. */
	private void spinUpAndFeed(double rps) {
		Robot.Shooter.setVelocity(rps);
		if (Robot.Shooter.isAtSpeed(rps)) {
			Robot.Spindexer.clockwise(Constants.Spindexer.kIndexingSpeed);
		} else {
			Robot.Spindexer.stop();
		}
	}

	public void registerNamedCommands() {

		NamedCommands.registerCommand(
				"ShootAtHub",
				Commands.defer(() -> shootAtHubCommand(Optional.empty()), Set.of()));

		NamedCommands.registerCommand(
				"PassLeft",
				Commands.defer(() -> passLeftCommand(Optional.empty()), Set.of()));

		NamedCommands.registerCommand(
				"PassRight",
				Commands.defer(() -> passRightCommand(Optional.empty()), Set.of()));

		NamedCommands.registerCommand(
				"PassClosest",
				Commands.defer(() -> passClosestCommand(Optional.empty()), Set.of()));
	}

	public Command aimAndShootCommand(
			TurretUtil.TargetType target,
			Optional<GenericHID> controller) {

		return Commands.parallel(

				Robot.Turret.autoAimCommand(
						() -> Robot.Drivetrain.getState().Pose,
						() -> target,
						() -> Robot.Drivetrain.getState().Speeds,
						() -> Units.degreesToRadians(
								Robot.Drivetrain.getPigeon2()
										.getAngularVelocityZWorld()
										.getValueAsDouble()),
						controller),

				Commands.sequence(
						Commands.waitSeconds(0.02),
						Commands.run(() -> {

							var pose = Robot.Drivetrain.getState().Pose;
							TurretUtil.ShotSolution solution = TurretUtil.computeShotSolution(pose, target);

							if (!solution.isValid) {
								Robot.Shooter.stop();
								Robot.Spindexer.stop();
								return;
							}

							SmartDashboard.putNumber("Shooter/TargetRPS", solution.shooterSpeedRPS);

							Robot.Shooter.setVelocity(solution.shooterSpeedRPS);

							boolean aligned = Math.abs(solution.robotRelativeAngleDegrees
										- Robot.Turret.getRobotRelativeAngleDegrees()) < kTurretAlignToleranceDeg;

							boolean atSpeed = Robot.Shooter.isAtSpeed(solution.shooterSpeedRPS);

							if (aligned && atSpeed) {
								Robot.Spindexer.clockwise(Constants.Spindexer.kIndexingSpeed);
							} else {
								Robot.Spindexer.stop();
							}

						}, Robot.Shooter, Robot.Spindexer))

						.finallyDo(__ -> {
							Robot.Shooter.stop();
							Robot.Spindexer.stop();
						}))
				.withName("AimAndShoot-" + target);
	}

	public Command shootWithoutAngleCheckCommand(
			TurretUtil.TargetType target,
			Optional<GenericHID> controller) {

		return Commands.run(() -> {

			var pose = Robot.Drivetrain.getState().Pose;

			TurretUtil.ShotSolution solution = TurretUtil.computeShotSolution(pose, target);

			Robot.Shooter.setVelocity(solution.shooterSpeedRPS);

			boolean turretSafe = solution.isValid
					&& Math.abs(solution.robotRelativeAngleDegrees
							- Robot.Turret.getRobotRelativeAngleDegrees()) < kTurretAlignToleranceDeg;

			if (turretSafe && Robot.Shooter.isAtSpeed(solution.shooterSpeedRPS)) {
				Robot.Spindexer.clockwise(Constants.Spindexer.kIndexingSpeed);
			} else {
				Robot.Spindexer.stop();
			}

		}, Robot.Shooter, Robot.Spindexer)

				.finallyDo(__ -> {

					Robot.Shooter.stop();
					Robot.Spindexer.stop();
				})
				.withName("ShootWithoutAngleCheck-" + target);
	}

	public Command shootAtSpeedWithoutAngleCheckCommand(
			double rps,
			Optional<GenericHID> controller) {

		return Commands.run(() -> {

			Robot.Shooter.setVelocity(rps);

			boolean turretSafe = Math.abs(Robot.Turret.getErrorDegrees()) < kTurretAlignToleranceDeg;

			if (turretSafe && Robot.Shooter.isAtSpeed(rps)) {
				Robot.Spindexer.clockwise(Constants.Spindexer.kIndexingSpeed);
			} else {
				Robot.Spindexer.stop();
			}

		}, Robot.Shooter, Robot.Spindexer)

				.finallyDo(__ -> {

					Robot.Shooter.stop();
					Robot.Spindexer.stop();
				});
	}

	public Command shootAtHubCommand(Optional<GenericHID> controller) {

		return aimAndShootCommand(
				TurretUtil.TargetType.AllianceHUB,
				controller)
				.withName("ShootAtHub");
	}

	public Command passLeftCommand(Optional<GenericHID> controller) {

		return aimAndShootCommand(
				TurretUtil.TargetType.LEFT_PASS,
				controller)
				.withName("PassLeft");
	}

	public Command passRightCommand(Optional<GenericHID> controller) {

		return aimAndShootCommand(
				TurretUtil.TargetType.RIGHT_PASS,
				controller)
				.withName("PassRight");
	}

	public Command passClosestCommand(Optional<GenericHID> controller) {

		Supplier<TurretUtil.TargetType> closestTarget = () -> TurretUtil.getClosestPassTarget(Robot.Drivetrain.getState().Pose);

		return Commands.parallel(

				Robot.Turret.autoAimCommand(
						() -> Robot.Drivetrain.getState().Pose,
						closestTarget,
						() -> Robot.Drivetrain.getState().Speeds,
						() -> Units.degreesToRadians(Robot.Drivetrain.getPigeon2()
								.getAngularVelocityZWorld().getValueAsDouble()),
						controller),

				Commands.sequence(
						Commands.waitSeconds(0.02),
						Commands.run(() -> {

							var pose = Robot.Drivetrain.getState().Pose;

							TurretUtil.ShotSolution solution = TurretUtil.computeShotSolution(pose,
									closestTarget.get());

							if (!solution.isValid) {
								Robot.Shooter.stop();
								Robot.Spindexer.stop();
								return;
							}

							SmartDashboard.putNumber("Shooter/TargetRPS", solution.shooterSpeedRPS);

							Robot.Shooter.setVelocity(solution.shooterSpeedRPS);

							boolean aligned = Math.abs(solution.robotRelativeAngleDegrees
									- Robot.Turret.getRobotRelativeAngleDegrees()) < kTurretAlignToleranceDeg;

							if (aligned && Robot.Shooter.isAtSpeed(solution.shooterSpeedRPS)) {
								Robot.Spindexer.clockwise(Constants.Spindexer.kIndexingSpeed);
							} else {
								Robot.Spindexer.stop();
							}

						}, Robot.Shooter, Robot.Spindexer))

						.finallyDo(__ -> {

							Robot.Shooter.stop();
							Robot.Spindexer.stop();
						}))
				.withName("PassClosest");
	}
}