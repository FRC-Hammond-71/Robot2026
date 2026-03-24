package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.units.measure.*;

import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.SubsystemWithMapleSimSimulation;
import frc.robot.util.dashboard.TurretUtil;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

import edu.wpi.first.wpilibj.RobotBase;

import java.util.Optional;
import java.util.function.Supplier;

public class TurretSubsystem extends SubsystemWithMapleSimSimulation {

	private final DCMotor dcMotor = DCMotor.getKrakenX60(1);

	private final int canID = Constants.Turret.kTurretCanID;
	private final double gearRatio = Constants.Turret.kGearRatio;

	private final double kP = Constants.Turret.kKP;
	private final double kI = Constants.Turret.kKI;
	private final double kD = Constants.Turret.kKD;

	private final double kS = Constants.Turret.kKS;
	private final double kV = Constants.Turret.kKV;
	private final double kA = Constants.Turret.kKA;

	private final boolean brakeMode = Constants.Turret.kBrakeMode;

	private final boolean enableStatorLimit = Constants.Turret.kEnableStatorLimit;
	private final double statorCurrentLimit = Constants.Turret.kStatorCurrentLimit;

	private final boolean enableSupplyLimit = Constants.Turret.kEnableSupplyLimit;
	private final double supplyCurrentLimit = Constants.Turret.kSupplyCurrentLimit;

	private final TalonFX motor;

	private TalonFXSimState simState;

	private final NeutralOut neutralOutRequest = new NeutralOut();
	private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);

	private final StatusSignal<Angle> positionSignal;
	private final StatusSignal<AngularVelocity> velocitySignal;
	private final StatusSignal<Voltage> voltageSignal;
	private final StatusSignal<Current> statorCurrentSignal;
	private final StatusSignal<Temperature> temperatureSignal;

	private final ArmFeedforward feedforward = new ArmFeedforward(kS, 0, kV, kA);

	private double targetRobotRelativeDegrees = Constants.Turret.kOriginAngle.in(Degrees);
	private double operatorOffsetDegrees = 0.0;

	private SingleJointedArmSim pivotSim;

	public TurretSubsystem(frc.robot.Robot robotInstance) {

		super(robotInstance);

		motor = new TalonFX(canID);

		positionSignal = motor.getPosition();
		velocitySignal = motor.getVelocity();
		voltageSignal = motor.getMotorVoltage();
		statorCurrentSignal = motor.getStatorCurrent();
		temperatureSignal = motor.getDeviceTemp();

		TalonFXConfiguration config = new TalonFXConfiguration();

		Slot0Configs slot0 = config.Slot0;
		slot0.kP = kP;
		slot0.kI = kI;
		slot0.kD = kD;
		slot0.kS = kS;
		slot0.kV = kV;
		slot0.kA = kA;

		CurrentLimitsConfigs currentLimits = config.CurrentLimits;
		currentLimits.StatorCurrentLimit = statorCurrentLimit;
		currentLimits.StatorCurrentLimitEnable = enableStatorLimit;
		currentLimits.SupplyCurrentLimit = supplyCurrentLimit;
		currentLimits.SupplyCurrentLimitEnable = enableSupplyLimit;

		config.MotorOutput.NeutralMode = brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;

		config.Feedback.RotorToSensorRatio = 1;
		config.Feedback.SensorToMechanismRatio = gearRatio;

		config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
		config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units
				.degreesToRotations(Constants.Turret.kMinAngleDegrees);

		config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
		config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units
				.degreesToRotations(Constants.Turret.kMaxAngleDegrees);

		motor.getConfigurator().apply(config);

		if (RobotBase.isSimulation()) {

			simState = motor.getSimState();

			pivotSim = new SingleJointedArmSim(
					dcMotor,
					gearRatio,
					0.01,
					0.1,
					Units.degreesToRadians(Constants.Turret.kMinAngleDegrees),
					Units.degreesToRadians(Constants.Turret.kMaxAngleDegrees),
					false,
					Units.degreesToRadians(Constants.Turret.kOriginAngle.in(Degrees)));

			// Sync raw rotor position with the origin angle BEFORE setPosition()
			// so that resetRobotRelativeAngle() creates no internal offset
			double originMechanismRotations = Units.degreesToRotations(
					Constants.Turret.kOriginAngle.in(Degrees));
			simState.setRawRotorPosition(originMechanismRotations * gearRatio);
		}

		resetRobotRelativeAngle();
	}

	protected void periodicReal() {

		BaseStatusSignal.refreshAll(
				positionSignal,
				velocitySignal,
				voltageSignal,
				statorCurrentSignal,
				temperatureSignal);
	}

	@Override
	public void simulationPeriodic() {

		if (!RobotBase.isSimulation())
			return;

		// Give the Talon the current battery voltage
		simState.setSupplyVoltage(RoboRioSim.getVInVoltage());

		// Voltage applied by Talon
		double motorVoltage = simState.getMotorVoltage();

		// Apply voltage to turret physics
		pivotSim.setInputVoltage(motorVoltage);

		// Advance physics simulation
		pivotSim.update(0.020);

		// Battery simulation
		RoboRioSim.setVInVoltage(
				BatterySim.calculateDefaultBatteryLoadedVoltage(
						pivotSim.getCurrentDrawAmps()));

		/*
		 * Convert turret angle → motor rotor position
		 */

		double turretRotations = Units.radiansToRotations(pivotSim.getAngleRads());

		double turretVelocityRotPerSec = Units.radiansToRotations(pivotSim.getVelocityRadPerSec());

		double rotorRotations = turretRotations * gearRatio;
		double rotorVelocity = turretVelocityRotPerSec * gearRatio;

		simState.setRawRotorPosition(rotorRotations);
		simState.setRotorVelocity(rotorVelocity);

		// Helpful debug
		SmartDashboard.putNumber("TurretSimAngleDeg",
				Units.radiansToDegrees(pivotSim.getAngleRads()));
	}

	public Rotation2d getDesiredHeadingFieldRelative(Rotation2d robotHeading) {
		return robotHeading.plus(Rotation2d.fromDegrees(targetRobotRelativeDegrees));
	}

	public void resetRobotRelativeAngle() {
		motor.setPosition(Constants.Turret.kOriginAngle);
	}

	public StatusSignal<Angle> getPositionSignal() {
		return positionSignal;
	}

	public double getPosition() {
		return positionSignal.getValueAsDouble();
	}

	public double getRobotRelativeAngleDegrees() {

		if (RobotBase.isSimulation()) {
			return Units.radiansToDegrees(pivotSim.getAngleRads());
		}

		return positionSignal.getValue().in(Degrees);
	}

	public double getFieldRelativeAngleDegrees(Rotation2d robotHeading) {

		return robotHeading
				.plus(Rotation2d.fromDegrees(getRobotRelativeAngleDegrees()))
				.getDegrees();
	}

	public static double fieldToRobotRelativeDegrees(Rotation2d fieldAngle, Rotation2d robotHeading) {

		// Target positions (getAllianceHub, etc.) are already alliance-specific,
		// so the field angle is already correct — no mirroring needed here.
		double deg = fieldAngle.minus(robotHeading).getDegrees();

		return deg < 0 ? deg + 360.0 : deg;
	}

	public double getTargetRobotRelativeAngleDegrees() {
		return targetRobotRelativeDegrees;
	}

	public double getOperatorOffsetDegrees() {
		return operatorOffsetDegrees;
	}

	public void adjustOperatorOffsetDegrees(double deltaDegrees) {
		operatorOffsetDegrees += deltaDegrees;
	}

	public double getErrorDegrees() {
		return targetRobotRelativeDegrees - getRobotRelativeAngleDegrees();
	}

	public void setRobotRelativeAngle(double angleDegrees) {
		setRobotRelativeAngle(angleDegrees, 0);
	}

	public void setRobotRelativeAngle(double angleDegrees, double feedForwardVolts) {

		if (!Constants.Turret.kTurretEnabled)
			return;

		angleDegrees += operatorOffsetDegrees;

		angleDegrees = Math.max(Constants.Turret.kMinAngleDegrees,
				Math.min(Constants.Turret.kMaxAngleDegrees, angleDegrees));

		targetRobotRelativeDegrees = angleDegrees;

		double positionRotations = Units.degreesToRotations(angleDegrees);

		motor.setControl(
			positionRequest.withPosition(positionRotations)
					// .withFeedForward(feedForwardVolts)
					);
	}

	public Command setRobotRelativeAngleCommand(double angleDegrees) {
		return runOnce(() -> setRobotRelativeAngle(angleDegrees));
	}

	public Command moveToRobotRelativeAngleCommand(double angleDegrees) {
		return run(() -> setRobotRelativeAngle(angleDegrees))
				.until(() -> Math.abs(angleDegrees - getRobotRelativeAngleDegrees()) < 2.0);
	}

	public Command stopCommand() {
		return runOnce(() -> setRobotRelativeAngle(getRobotRelativeAngleDegrees()));
	}

	public Command neutralOutputCommand() {
		return run(() -> motor.setControl(neutralOutRequest));
	}

	public Command autoAimCommand(
			Supplier<Pose2d> robotPoseSupplier,
			Supplier<TurretUtil.TargetType> target,
			Supplier<ChassisSpeeds> speedsSupplier,
			Supplier<Double> gyroRateRadPerSec,
			Optional<GenericHID> controller) {

		return run(() -> {

			Pose2d robotPose = robotPoseSupplier.get();
			ChassisSpeeds speeds = speedsSupplier.get();

			// Use whichever angular rate source reports higher magnitude
			double omegaPigeon = gyroRateRadPerSec.get();
			double omegaKinematics = speeds.omegaRadiansPerSecond;
			double omega = Math.abs(omegaPigeon) > Math.abs(omegaKinematics)
					? omegaPigeon : omegaKinematics;

			TurretUtil.ShotSolution solution = Constants.Turret.kShootOnTheMove
					? TurretUtil.computeLeadingShot(robotPose, speeds, target.get())
					: TurretUtil.computeShotSolution(robotPose, target.get());

			if (solution.isValid) {

				Translation2d turretField = TurretUtil.getTurretPose(robotPose).getTranslation();
				Translation2d goal = TurretUtil.getTargetPose(target.get()).getTranslation();
				double dx = goal.getX() - turretField.getX();
				double dy = goal.getY() - turretField.getY();
				double distSq = dx * dx + dy * dy;

				// Robot center field-relative velocity
				double cos = robotPose.getRotation().getCos();
				double sin = robotPose.getRotation().getSin();
				double vxRobotField = speeds.vxMetersPerSecond * cos - speeds.vyMetersPerSecond * sin;
				double vyRobotField = speeds.vxMetersPerSecond * sin + speeds.vyMetersPerSecond * cos;

				// Turret pivot orbits robot center: v_turret = v_robot + ω × offset
				double offsetXField = cos * Constants.Turret.kTurretOffsetX - sin * Constants.Turret.kTurretOffsetY;
				double offsetYField = sin * Constants.Turret.kTurretOffsetX + cos * Constants.Turret.kTurretOffsetY;
				double vxTurret = vxRobotField - omega * offsetYField;
				double vyTurret = vyRobotField + omega * offsetXField;

				// dθ/dt of field angle from turret pivot to target
				double dThetaDt = (-dy * vxTurret + dx * vyTurret) / distSq;

				// Total turret rate in robot frame = field angle rate - robot rotation
				double turretRateRadPerSec = dThetaDt - omega;
				double turretVelRotPerSec = turretRateRadPerSec / (2.0 * Math.PI);
				double ffVolts = kV * turretVelRotPerSec;

				setRobotRelativeAngle(solution.robotRelativeAngleDegrees, ffVolts);
				
			} else if (controller.isPresent()) {
				// controller.get().setRumble(RumbleType.kBothRumble, 1);
			}

			SmartDashboard.putBoolean("Turret/AutoAim/IsValid", solution.isValid);

		}).finallyDo(() -> {

			// controller.ifPresent(c -> c.setRumble(RumbleType.kBothRumble, 0));

		}).withName("AutoAim-" + target.toString());
	}

	@Override
	protected void periodicSimulated() {
	}

	@Override
	protected void periodicAny() {

		SmartDashboard.putNumber("Turret/Error", getErrorDegrees());
		SmartDashboard.putNumber("Turret/OperatorOffsetDegrees", operatorOffsetDegrees);

	}
}