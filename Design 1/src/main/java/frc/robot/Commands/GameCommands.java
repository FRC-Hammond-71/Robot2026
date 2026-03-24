package frc.robot.Commands;

import java.util.Optional;
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

	private static final double kTurretAlignToleranceDeg = 2;

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
				shootAtHubCommand(Optional.empty()));
				
		NamedCommands.registerCommand(
				"PassLeft",
				passLeftCommand(Optional.empty()));

		NamedCommands.registerCommand(
				"PassRight",
				passRightCommand(Optional.empty()));

		NamedCommands.registerCommand(
				"PassClosest",
				passClosestCommand(Optional.empty()));
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

					boolean aligned = Math
							.abs(Robot.Turret.getErrorDegrees()) < kTurretAlignToleranceDeg;

					boolean atSpeed = Robot.Shooter.isAtSpeed(solution.shooterSpeedRPS);

					if (aligned && atSpeed) {
						Robot.Spindexer.clockwise(Constants.Spindexer.kIndexingSpeed);
					} else {
						Robot.Spindexer.stop();
					}

				}, Robot.Shooter, Robot.Spindexer)

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

			spinUpAndFeed(solution.shooterSpeedRPS);

			// controller.ifPresent(
			// c -> c.setRumble(RumbleType.kBothRumble, 0));

		}, Robot.Shooter, Robot.Spindexer)

				.finallyDo(__ -> {

					Robot.Shooter.stop();
					Robot.Spindexer.stop();

					// controller.ifPresent(
					// c -> c.setRumble(RumbleType.kBothRumble, 0));
				})
				.withName("ShootWithoutAngleCheck-" + target);
	}

	public Command shootAtSpeedWithoutAngleCheckCommand(
			double rps,
			Optional<GenericHID> controller) {

		return Commands.run(() -> {

			spinUpAndFeed(rps);

		}, Robot.Shooter, Robot.Spindexer)

				.finallyDo(__ -> {

					Robot.Shooter.stop();
					Robot.Spindexer.stop();

					// controller.ifPresent(
					// c -> c.setRumble(RumbleType.kBothRumble, 0));
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

		Supplier<TurretUtil.TargetType> closestTarget = () -> TurretUtil
				.getClosestPassTarget(Robot.Drivetrain.getState().Pose);

		return Commands.parallel(

				Robot.Turret.autoAimCommand(
						() -> Robot.Drivetrain.getState().Pose,
						closestTarget,
						() -> Robot.Drivetrain.getState().Speeds,
						() -> Units.degreesToRadians(Robot.Drivetrain.getPigeon2()
								.getAngularVelocityZWorld().getValueAsDouble()),
						controller),

				Commands.run(() -> {

					var pose = Robot.Drivetrain.getState().Pose;

					TurretUtil.ShotSolution solution = TurretUtil.computeShotSolution(pose,
							closestTarget.get());

					SmartDashboard.putNumber("Shooter/TargetRPS", solution.shooterSpeedRPS);

					boolean aligned = solution.isValid
							&& Math.abs(Robot.Turret
									.getErrorDegrees()) < kTurretAlignToleranceDeg;

					if (aligned) {
						spinUpAndFeed(solution.shooterSpeedRPS);
					} else {
						Robot.Shooter.setVelocity(solution.shooterSpeedRPS);
						Robot.Spindexer.stop();
					}

				}, Robot.Shooter, Robot.Spindexer)

						.finallyDo(__ -> {

							Robot.Shooter.stop();
							Robot.Spindexer.stop();
						}))
				.withName("PassClosest");
	}
}