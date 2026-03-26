package frc.robot.Commands;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Robot;

public class PreflightCheckCommand extends Command {

	private final Robot robot;

	private final PositionVoltage steerPositionRequest = new PositionVoltage(0);
	private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
	private final NeutralOut neutralOut = new NeutralOut();

	private enum Phase {
		DRIVETRAIN_MODULE,
		TURRET_POSITIONS,
		TURRET_SWEEP,
		TURRET_RETURN,
		SHOOTER,
		SPINDEXER,
		INTAKE_TOGGLE,
		DONE
	}

	private Phase phase;
	private int moduleIndex;
	private int moduleStep;
	private double phaseStartTime;

	private static final double[] TURRET_TARGETS = { 90, 135, 180, 225, 270 };
	private int turretTargetIndex;

	private static final double[] SHOOTER_DISTANCES = { 2, 3, 4, 5, 6 };
	private int shooterDistanceIndex;
	private double shooterTargetRPS;

	private int spindexerStep;
	private int intakeStep;


	public PreflightCheckCommand(Robot robot) {
		this.robot = robot;
		addRequirements(robot.Drivetrain, robot.Turret, robot.Shooter, robot.Spindexer, robot.Intake);
	}

	@Override
	public void initialize() {
		phase = Phase.DRIVETRAIN_MODULE;
		moduleIndex = 0;
		moduleStep = 0;
		turretTargetIndex = 0;
		shooterDistanceIndex = 0;
		spindexerStep = 0;
		intakeStep = 0;
		phaseStartTime = Timer.getFPGATimestamp();
	}

	@Override
	public void execute() {
		double elapsed = Timer.getFPGATimestamp() - phaseStartTime;

		switch (phase) {
			case DRIVETRAIN_MODULE:
				executeDrivetrainModule(elapsed);
				break;
			case TURRET_POSITIONS:
				executeTurretPositions(elapsed);
				break;
			case TURRET_SWEEP:
				executeTurretSweep(elapsed);
				break;
			case TURRET_RETURN:
				executeTurretReturn(elapsed);
				break;
			case SHOOTER:
				executeShooter(elapsed);
				break;
			case SPINDEXER:
				executeSpindexer(elapsed);
				break;
			case INTAKE_TOGGLE:
				executeIntakeToggle(elapsed);
				break;
			case DONE:
				break;
		}
	}

	private void executeDrivetrainModule(double elapsed) {
		var module = robot.Drivetrain.getModule(moduleIndex);

		switch (moduleStep) {
			case 0:
				double targetSteerPos = module.getSteerMotor().getPosition().getValueAsDouble() + 0.25;
				module.getSteerMotor().setControl(steerPositionRequest.withPosition(targetSteerPos));
				moduleStep = 1;
				phaseStartTime = Timer.getFPGATimestamp();
				break;

			case 1:
				if (elapsed >= 0.5) {
					module.getDriveMotor().setControl(driveDutyCycle.withOutput(0.15));
					moduleStep = 2;
					phaseStartTime = Timer.getFPGATimestamp();
				}
				break;

			case 2:
				if (elapsed >= 0.25) {
					module.getDriveMotor().setControl(neutralOut);
					moduleIndex++;
					moduleStep = 0;
					phaseStartTime = Timer.getFPGATimestamp();

					if (moduleIndex >= 4) {
						for (int i = 0; i < 4; i++) {
							robot.Drivetrain.getModule(i).getSteerMotor().setControl(neutralOut);
						}
						advanceTo(Phase.TURRET_POSITIONS);
					}
				}
				break;
		}
	}

	private void executeTurretPositions(double elapsed) {
		double target = TURRET_TARGETS[turretTargetIndex];
		robot.Turret.setRobotRelativeAngle(target);

		boolean atTarget = Math.abs(robot.Turret.getRobotRelativeAngleDegrees() - target) < 3.0;
		if (atTarget || elapsed >= 1.5) {
			turretTargetIndex++;
			phaseStartTime = Timer.getFPGATimestamp();

			if (turretTargetIndex >= TURRET_TARGETS.length) {
				advanceTo(Phase.TURRET_SWEEP);
			}
		}
	}

	private void executeTurretSweep(double elapsed) {
		double t = Math.min(elapsed / 2.0, 1.0);
		double angle = 90 + (270 - 90) * t;
		robot.Turret.setRobotRelativeAngle(angle);

		if (elapsed >= 3.0) {
			advanceTo(Phase.TURRET_RETURN);
		}
	}

	private void executeTurretReturn(double elapsed) {
		robot.Turret.setRobotRelativeAngle(180);

		if (Math.abs(robot.Turret.getRobotRelativeAngleDegrees() - 180) < 3.0 || elapsed >= 1.5) {
			advanceTo(Phase.SHOOTER);
		}
	}

	private void executeShooter(double elapsed) {
		if (elapsed < 0.05) {
			shooterTargetRPS = robot.Shooter.calculateTargetRPS(SHOOTER_DISTANCES[shooterDistanceIndex]);
			robot.Shooter.setVelocity(shooterTargetRPS);
		}

		if (elapsed >= 0.5) {
			shooterDistanceIndex++;
			phaseStartTime = Timer.getFPGATimestamp();

			if (shooterDistanceIndex >= SHOOTER_DISTANCES.length) {
				robot.Shooter.stop();
				advanceTo(Phase.SPINDEXER);
			}
		}
	}

	private void executeSpindexer(double elapsed) {
		if (spindexerStep == 0) {
			if (elapsed < 0.05) {
				robot.Spindexer.clockwise(0.3);
			}
			if (elapsed >= 0.5) {
				robot.Spindexer.stop();
				spindexerStep = 1;
				phaseStartTime = Timer.getFPGATimestamp();
			}
		} else {
			if (elapsed < 0.05) {
				robot.Spindexer.counterclockwise(0.3);
			}
			if (elapsed >= 0.5) {
				robot.Spindexer.stop();
				advanceTo(Phase.INTAKE_TOGGLE);
			}
		}
	}

	private void executeIntakeToggle(double elapsed) {
		if (intakeStep == 0) {
			if (elapsed < 0.05) {
				if (robot.Intake.isExtended) {
					robot.Intake.toggleExtension();
					intakeStep = -1;
					return;
				}
				robot.Intake.toggleExtension();
			}
			if (!robot.Intake.isExtensionMoving() && elapsed >= 0.2) {
				intakeStep = 1;
				phaseStartTime = Timer.getFPGATimestamp();
			}
		} else if (intakeStep == -1) {
			if (!robot.Intake.isExtensionMoving() && elapsed >= 0.2) {
				robot.Intake.toggleExtension();
				intakeStep = 0;
				phaseStartTime = Timer.getFPGATimestamp();
			}
		} else {
			if (elapsed < 0.05) {
				robot.Intake.toggleExtension();
			}
			if (!robot.Intake.isExtensionMoving() && elapsed >= 0.2) {
				advanceTo(Phase.DONE);
			}
		}
	}

	private void advanceTo(Phase next) {
		phase = next;
		phaseStartTime = Timer.getFPGATimestamp();
	}

	@Override
	public boolean isFinished() {
		return phase == Phase.DONE;
	}

	@Override
	public void end(boolean interrupted) {
		for (int i = 0; i < 4; i++) {
			robot.Drivetrain.getModule(i).getSteerMotor().setControl(neutralOut);
			robot.Drivetrain.getModule(i).getDriveMotor().setControl(neutralOut);
		}
		robot.Turret.setRobotRelativeAngle(180);
		robot.Shooter.stop();
		robot.Spindexer.stop();
		if (robot.Intake.isExtended) {
			robot.Intake.toggleExtension();
		}
	}
}
