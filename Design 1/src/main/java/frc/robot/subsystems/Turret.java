package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.dashboard.TurretUtil;
import edu.wpi.first.wpilibj.DriverStation;


import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Turret subsystem using TalonFX with Krakenx60 motor.
 * All angles are robot-relative degrees (0° = robot forward, + = left/CCW, - = right/CW).
 * Physical range: [90°, 270°]. Origin (startup position) = 180°.
 */
public class Turret extends SubsystemBase {

  // Constants
  private final DCMotor dcMotor = DCMotor.getKrakenX60(1);
  private final int canID = Constants.Turret.kTurretCanID;
  private final double gearRatio = Constants.Turret.kGearRatio;
  private final double kP = Constants.Turret.kKP;
  private final double kI = Constants.Turret.kKI;
  private final double kD = Constants.Turret.kKD;
  private final double kS = Constants.Turret.kKS;
  private final double kV = Constants.Turret.kKV;
  private final double kA = Constants.Turret.kKA;
  private final double kG = Constants.Turret.kKG;
  private final double maxVelocity = Constants.Turret.kMaxVelocity;
  private final double maxAcceleration = Constants.Turret.kMaxAcceleration;
  private final boolean brakeMode = Constants.Turret.kBrakeMode;
  private final boolean enableStatorLimit = Constants.Turret.kEnableStatorLimit;
  private final double statorCurrentLimit = Constants.Turret.kStatorCurrentLimit;
  private final boolean enableSupplyLimit = Constants.Turret.kEnableSupplyLimit;
  private final double supplyCurrentLimit = Constants.Turret.kSupplyCurrentLimit;

  // Feedforward
  private final ArmFeedforward feedforward = new ArmFeedforward(
    kS, // kS
    0, // kG - Pivot doesn't need gravity compensation
    kV, // kV
    kA // kA
  );

  // Motor controller
  private final TalonFX motor;
  private final NeutralOut neutralOutRequest;
  private final PositionVoltage positionRequest;
  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Voltage> voltageSignal;
  private final StatusSignal<Current> statorCurrentSignal;
  private final StatusSignal<Temperature> temperatureSignal;

  // Target tracking for telemetry
  private double targetRobotRelativeDegrees = Constants.Turret.kOriginAngle.in(Degrees);

  // Simulation
  private final SingleJointedArmSim pivotSim;

  /**
   * Creates a new Turret Subsystem.
   */
  public Turret() {
    // Initialize motor controller
    motor = new TalonFX(canID);

    // Create control requests
    neutralOutRequest = new NeutralOut();
    positionRequest = new PositionVoltage(0).withSlot(0);

    // get status signals
    positionSignal = motor.getPosition();
    velocitySignal = motor.getVelocity();
    voltageSignal = motor.getMotorVoltage();
    statorCurrentSignal = motor.getStatorCurrent();
    temperatureSignal = motor.getDeviceTemp();

    TalonFXConfiguration config = new TalonFXConfiguration();

    // config.ClosedLoopGeneral.Con

    // config.ClosedLoopRamps.`/

    // Configure PID for slot 0
    Slot0Configs slot0 = config.Slot0;
    // slot0.ClosedLoopErrorThreshold = 
    slot0.kP = kP;
    slot0.kI = kI;
    slot0.kD = kD;
    slot0.kS = kS;
    slot0.kV = kV;
    slot0.kA = kA;

    // Set current limits
    CurrentLimitsConfigs currentLimits = config.CurrentLimits;
    currentLimits.StatorCurrentLimit = statorCurrentLimit;
    currentLimits.StatorCurrentLimitEnable = enableStatorLimit;
    currentLimits.SupplyCurrentLimit = supplyCurrentLimit;
    currentLimits.SupplyCurrentLimitEnable = enableSupplyLimit;

    // Set brake mode
    config.MotorOutput.NeutralMode = brakeMode
      ? NeutralModeValue.Brake
      : NeutralModeValue.Coast;

    config.Feedback.RotorToSensorRatio = 1; // motor rotations per output (CANcoder) rotation
    config.Feedback.SensorToMechanismRatio = gearRatio;   // CANcoder is on the output shaft

    // Enforce angle limits in hardware so PID cannot drive outside the range
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.degreesToRotations(Constants.Turret.kMinAngleDegrees);
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.degreesToRotations(Constants.Turret.kMaxAngleDegrees);

    // Apply configuration
    motor.getConfigurator().apply(config);

    // Initialize simulation
    pivotSim = new SingleJointedArmSim(
      dcMotor, // Motor type
      gearRatio,
      0.01, // Arm moment of inertia - Small value since there are no arm parameters
      0.1, // Arm length (m) - Small value since there are no arm parameters
      Units.degreesToRadians(Constants.Turret.kMinAngleDegrees), // Min angle (rad)
      Units.degreesToRadians(Constants.Turret.kMaxAngleDegrees), // Max angle (rad)
      false, // Simulate gravity - Disable gravity for pivot
      Units.degreesToRadians(0) // Starting position (rad)
    );

    this.resetRobotRelativeAngle();
  }

  @Override
  public void periodic() {
    
    BaseStatusSignal.refreshAll(
      positionSignal,
      velocitySignal,
      voltageSignal,
      statorCurrentSignal,
      temperatureSignal
    );

    // SmartDashboard.putNumber("Turret/CurrentAngle", getRobotRelativeAngleDegrees());
    // SmartDashboard.putNumber("Turret/DesiredAngle", targetRobotRelativeDegrees);
    // SmartDashboard.putNumber("Turret/ErrorAngle", getErrorDegrees());
  }

  public Rotation2d getDesiredHeadingFieldRelative(Rotation2d robotHeading)
  {
    return robotHeading.plus(Rotation2d.fromDegrees(targetRobotRelativeDegrees));
  }

  public void resetRobotRelativeAngle()
  {
    this.motor.setPosition(Constants.Turret.kOriginAngle);
  }

  @Override
  public void simulationPeriodic() {
    pivotSim.setInput(motor.getSimState().getMotorVoltage());

    pivotSim.update(0.020);
    RoboRioSim.setVInVoltage(
      BatterySim.calculateDefaultBatteryLoadedVoltage(
        pivotSim.getCurrentDrawAmps()
      )
    );

    double motorPosition = Radians.of(pivotSim.getAngleRads() * gearRatio).in(
      Rotations
    );
    double motorVelocity = RadiansPerSecond.of(
      pivotSim.getVelocityRadPerSec() * gearRatio
    ).in(RotationsPerSecond);

    motor.getSimState().setRawRotorPosition(motorPosition);
    motor.getSimState().setRotorVelocity(motorVelocity);
  }

  /**
   * Get the raw position StatusSignal (in Rotations) for buffering.
   */
  public StatusSignal<Angle> getPositionSignal() {
    return positionSignal;
  }

  /**
   * Get the current position in Rotations (raw motor value, includes origin offset).
   * @return Position in Rotations
   */
  public double getPosition() {
    return positionSignal.getValueAsDouble();
  }

  /**
   * Get the turret angle in robot-relative degrees.
   * @return Angle in degrees (0° = robot forward, + = left/CCW, range [90°, 270°])
   */
  public double getRobotRelativeAngleDegrees() {
    return positionSignal.getValue().in(Degrees);
  }

  /**
   * Get the turret's field-relative heading in degrees.
   * Field heading = robot heading + turret robot-relative angle.
   * @param robotHeading The current robot heading on the field
   * @return Field-relative turret heading in degrees
   */
  public double getFieldRelativeAngleDegrees(Rotation2d robotHeading) {
    return robotHeading.plus(Rotation2d.fromDegrees(getRobotRelativeAngleDegrees())).getDegrees();
  }

  /**
   * Convert a field-relative angle to robot-relative turret degrees [0°, 360°).
   * Uses Rotation2d subtraction for correct circular arithmetic.
   * @param fieldAngle  The field-relative direction (e.g. angle to target)
   * @param robotHeading The current robot heading
   * @return Robot-relative angle in [0°, 360°), directly comparable to turret motor range [90°, 270°]
   */

  public static double fieldToRobotRelativeDegrees(Rotation2d fieldAngle, Rotation2d robotHeading) {
   //WORKS ON BLUE
    //double deg = fieldAngle.minus(robotHeading).getDegrees(); // (-180°, 180°]
    //return deg < 0 ? deg + 360.0 : deg;                      // [0°, 360°)

    
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
        fieldAngle = new Rotation2d(Math.PI - fieldAngle.getRadians());
    }
    double deg = fieldAngle.minus(robotHeading).getDegrees();
    return deg < 0 ? deg + 360.0 : deg;
}


  /**
   * Get the current velocity in rotations per second.
   */
  public double getVelocity() {
    return velocitySignal.getValueAsDouble();
  }

  /**
   * Get the current applied voltage.
   */
  public double getVoltage() {
    return voltageSignal.getValueAsDouble();
  }

  /**
   * Get the current motor current in amps.
   */
  public double getCurrent() {
    return statorCurrentSignal.getValueAsDouble();
  }

  /**
   * Get the current motor temperature in Celsius.
   */
  public double getTemperature() {
    return temperatureSignal.getValueAsDouble();
  }

  /**
   * Get the target angle in robot-relative degrees.
   */
  public double getTargetRobotRelativeAngleDegrees() {
    return targetRobotRelativeDegrees;
  }

  /**
   * Get the position error in degrees (target - current).
   */
  public double getErrorDegrees() {
    return targetRobotRelativeDegrees - getRobotRelativeAngleDegrees();
  }

  /**
   * Set turret angle.
   * @param angleDegrees The target angle in degrees (0 = forward)
   */
  public void setRobotRelativeAngle(double angleDegrees) {
    setRobotRelativeAngle(angleDegrees, 0);
  }

  /**
   * Set turret angle with feedforward voltage.
   * @param angleDegrees The target angle in degrees (0 = forward)
   * @param feedForwardVolts Extra voltage to apply (e.g. yaw-rate compensation)
   */
  public void setRobotRelativeAngle(double angleDegrees, double feedForwardVolts) {
    if (!Constants.Turret.kTurretEnabled) {
      return;
    }

    this.targetRobotRelativeDegrees = angleDegrees;

    // Convert degrees to motor rotations, accounting for the 180° origin offset
    double positionRotations = Units.degreesToRotations(angleDegrees);

    motor.setControl(positionRequest.withPosition(positionRotations).withFeedForward(feedForwardVolts));
  }

  /**
   * Get the pivot simulation for testing.
   */
  public SingleJointedArmSim getSimulation() {
    return pivotSim;
  }

  /**
   * Creates a command to set the turret to a specific angle.
   * @param angleDegrees The target angle in degrees
   */
  public Command setRobotRelativeAngleCommand(double angleDegrees) {
    return runOnce(() -> setRobotRelativeAngle(angleDegrees));
  }

  /**
   * Creates a command to move the turret to a specific angle using position control.
   * @param angleDegrees The target angle in degrees
   */
  public Command moveToRobotRelativeAngleCommand(double angleDegrees) {
    return run(() -> setRobotRelativeAngle(angleDegrees))
      .until(() -> Math.abs(angleDegrees - getRobotRelativeAngleDegrees()) < 2.0);
  }

  /**
   * Creates a command to hold the turret at its current position.
   */
  public Command stopCommand() {
    return runOnce(() -> setRobotRelativeAngle(getRobotRelativeAngleDegrees()));
  }

  /**
   * Creates a command that applies neutral output (no commanded motion).
   */
  public Command neutralOutputCommand() {
    return run(() -> motor.setControl(neutralOutRequest));
  }

  /**
   * Creates a command to automatically aim the turret at a target.
   * @param robotPoseSupplier Supplier that provides the current robot pose
   * @param target The target to aim at (HUB, LEFT_PASS, or RIGHT_PASS)
   */
  /**
   * Auto-aim without feedforward (for auto paths / named commands).
   */
  public Command autoAimCommand(Supplier<Pose2d> robotPoseSupplier, Supplier<TurretUtil.TargetType> target) {
    return autoAimCommand(robotPoseSupplier, target, () -> 0.0);
  }

  /**
   * Auto-aim with yaw-rate feedforward for reduced tracking delay.
   * @param robotPoseSupplier Supplier for the current robot pose
   * @param target The target to aim at
   * @param robotOmegaRadPerSec Supplier for the robot's angular velocity (rad/s, CCW+)
   */
  public Command autoAimCommand(Supplier<Pose2d> robotPoseSupplier, Supplier<TurretUtil.TargetType> target,
                                 DoubleSupplier robotOmegaRadPerSec) {
    return run(() -> {
      Pose2d robotPose = robotPoseSupplier.get();

      TurretUtil.ShotSolution solution = TurretUtil.computeShotSolution(robotPose, target.get());

      if (solution.isValid) {
        // Feedforward: when robot rotates at ω, turret target moves at -ω in turret frame
        double omega = robotOmegaRadPerSec.getAsDouble();
        double turretVelRotPerSec = -omega / (2.0 * Math.PI);
        double ffVolts = kV * turretVelRotPerSec;
        setRobotRelativeAngle(solution.robotRelativeAngleDegrees, ffVolts);

        SmartDashboard.putBoolean("Turret/AutoAim/IsValid", true);
      }
      else
      {
        SmartDashboard.putBoolean("Turret/AutoAim/IsValid", false);
      }

    }).withName("AutoAim-" + target.toString());
  }

  //------------------------ Tuning -----------------------//

  /**
   * Sets turret angle using tunable PID values and setpoint from dashboard.
   */
  private void turretTunable() {
    double kP = SmartDashboard.getNumber("Turret/kP", 0);
    double kI = SmartDashboard.getNumber("Turret/kI", 0);
    double kD = SmartDashboard.getNumber("Turret/kD", 0);
    double kV = SmartDashboard.getNumber("Turret/kV", 0);
    double kA = SmartDashboard.getNumber("Turret/kA", 0);
    double angleSetpoint = SmartDashboard.getNumber("Turret/AngleSetpoint", 0);

    Slot0Configs slot0 = new Slot0Configs();
    slot0.kP = kP;
    slot0.kI = kI;
    slot0.kD = kD;
    slot0.kV = kV;
    slot0.kA = kA;
    slot0.kS = kS;

    motor.getConfigurator().apply(slot0);

    setRobotRelativeAngle(angleSetpoint);
  }
}
