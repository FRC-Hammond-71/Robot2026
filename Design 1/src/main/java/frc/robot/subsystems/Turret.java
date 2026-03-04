package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;


import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
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

import java.util.function.Supplier;

/**
 * Pivot subsystem using TalonFX with Krakenx60 motor
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

  // WCP Throughbore CANcoder on output shaft
  private final CANcoder m_cancoder = new CANcoder(Constants.Turret.kThroughboreCanID);

  // Motor controller
  private final TalonFX motor;
  private final PositionVoltage positionRequest;
  private final VelocityVoltage velocityRequest;
  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Voltage> voltageSignal;
  private final StatusSignal<Current> statorCurrentSignal;
  private final StatusSignal<Temperature> temperatureSignal;

  // Target tracking for telemetry
  private double targetAngleDegrees = 0.0;
  private double targetVelocityDegPerSec = 0.0;

  // Simulation
  private final SingleJointedArmSim pivotSim;

  /**
   * Creates a new Pivot Subsystem.
   */
  public Turret() {
    // Initialize motor controller
    motor = new TalonFX(canID);

    // Create control requests
    positionRequest = new PositionVoltage(0).withSlot(0);
    velocityRequest = new VelocityVoltage(0).withSlot(0);

    // get status signals
    positionSignal = motor.getPosition();
    velocitySignal = motor.getVelocity();
    voltageSignal = motor.getMotorVoltage();
    statorCurrentSignal = motor.getStatorCurrent();
    temperatureSignal = motor.getDeviceTemp();

    TalonFXConfiguration config = new TalonFXConfiguration();

    // Configure PID for slot 0
    Slot0Configs slot0 = config.Slot0;
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

    // Configure WCP Throughbore CANcoder
    CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.MagnetOffset = Constants.Turret.kEncoderOffset;
    m_cancoder.getConfigurator().apply(cancoderConfig);

    // Fuse CANcoder as feedback source — TalonFX uses absolute position from CANcoder
    // combined with internal encoder velocity for high-rate closed-loop
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    config.Feedback.FeedbackRemoteSensorID = Constants.Turret.kThroughboreCanID;
    config.Feedback.RotorToSensorRatio = gearRatio; // motor rotations per output (CANcoder) rotation
    config.Feedback.SensorToMechanismRatio = 1.0;   // CANcoder is on the output shaft

    // Enforce angle limits in hardware so PID cannot drive outside the range
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.Turret.kMinAngleDegrees / 360.0;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Turret.kMaxAngleDegrees / 360.0;

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
  }

  /**
   * Update simulation and telemetry.
   */
  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(
      positionSignal,
      velocitySignal,
      voltageSignal,
      statorCurrentSignal,
      temperatureSignal
    );
  }

  /**
   * Update simulation.
   */
  @Override
  public void simulationPeriodic() {
    // Set input voltage from motor controller to simulation
    // Note: This may need to be talonfx.getSimState().getMotorVoltage() as the input
    //pivotSim.setInput(dcMotor.getVoltage(dcMotor.getTorque(pivotSim.getCurrentDrawAmps()), pivotSim.getVelocityRadPerSec()));
    // pivotSim.setInput(getVoltage());
    // Set input voltage from motor controller to simulation
    // Use motor voltage for TalonFX simulation input
    pivotSim.setInput(motor.getSimState().getMotorVoltage());

    // Update simulation by 20ms
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
   * Get the current position in Rotations.
   * @return Position in Rotations
   */
  @Logged(name = "Position/Rotations")
  public double getPosition() {
    // Rotations
    return positionSignal.getValueAsDouble();
  }

  /**
   * Get the current velocity in rotations per second.
   * @return Velocity in rotations per second
   */
  @Logged(name = "Velocity")
  public double getVelocity() {
    return velocitySignal.getValueAsDouble();
  }

  /**
   * Get the current applied voltage.
   * @return Applied voltage
   */
  @Logged(name = "Voltage")
  public double getVoltage() {
    return voltageSignal.getValueAsDouble();
  }

  /**
   * Get the current motor current.
   * @return Motor current in amps
   */
  public double getCurrent() {
    return statorCurrentSignal.getValueAsDouble();
  }

  /**
   * Get the current motor temperature.
   * @return Motor temperature in Celsius
   */
  public double getTemperature() {
    return temperatureSignal.getValueAsDouble();
  }

  /**
   * Get the target angle in degrees.
   * @return Target angle in degrees
   */
  @Logged(name = "Target/AngleDegrees")
  public double getTargetAngleDegrees() {
    return targetAngleDegrees;
  }

  /**
   * Get the target velocity in degrees per second.
   * @return Target velocity in degrees per second
   */
  @Logged(name = "Target/VelocityDegPerSec")
  public double getTargetVelocityDegPerSec() {
    return targetVelocityDegPerSec;
  }

  /**
   * Get the position error in degrees (target - current).
   * @return Position error in degrees
   */
  @Logged(name = "Error/AngleDegrees")
  public double getErrorDegrees() {
    double currentAngleDegrees = Units.rotationsToDegrees(getPosition());
    return targetAngleDegrees - currentAngleDegrees;
  }

  /**
   * Set pivot angle.
   * @param angleDegrees The target angle in degrees
   */
  public void setAngle(double angleDegrees) {
    setAngle(angleDegrees, 0);
  }

  /**
   * Set pivot angle with acceleration.
   * @param angleDegrees The target angle in degrees
   * @param acceleration The acceleration in rad/s²
   */
  public void setAngle(double angleDegrees, double acceleration) {
    // Clamp to allowed range before sending to motor
    angleDegrees = Math.max(Constants.Turret.kMinAngleDegrees, Math.min(Constants.Turret.kMaxAngleDegrees, angleDegrees));

    // Track target for telemetry (keep desired angle so getErrorDegrees() reflects true misalignment)
    this.targetAngleDegrees = angleDegrees;
    this.targetVelocityDegPerSec = 0;

    if (!Constants.Turret.kTurretEnabled) {
      // Hold at minimum angle (closest to forward) when turret is disabled
      motor.setControl(positionRequest.withPosition(Constants.Turret.kMinAngleDegrees / 360.0));
      return;
    }

    // Convert degrees to rotations
    double angleRadians = Units.degreesToRadians(angleDegrees);
    double positionRotations = angleRadians / (2.0 * Math.PI);

    motor.setControl(positionRequest.withPosition(positionRotations));
  }

  /**
   * Set pivot angular velocity.
   * @param velocityDegPerSec The target velocity in degrees per second
   */
  public void setVelocity(double velocityDegPerSec) {
    setVelocity(velocityDegPerSec, 0);
  }

  /**
   * Set pivot angular velocity with acceleration.
   * @param velocityDegPerSec The target velocity in degrees per second
   * @param acceleration The acceleration in degrees per second squared
   */
  public void setVelocity(double velocityDegPerSec, double acceleration) {
    // Track target for telemetry
    this.targetVelocityDegPerSec = velocityDegPerSec;
    this.targetAngleDegrees = Units.rotationsToDegrees(getPosition());

    if (!Constants.Turret.kTurretEnabled) {
      motor.setControl(positionRequest.withPosition(Constants.Turret.kMinAngleDegrees / 360.0));
      return;
    }

    // Convert degrees/sec to rotations/sec
    double velocityRadPerSec = Units.degreesToRadians(velocityDegPerSec);
    double velocityRotations = velocityRadPerSec / (2.0 * Math.PI);

    motor.setControl(velocityRequest.withVelocity(velocityRotations));
  }

  /**
   * Set motor voltage directly.
   * @param voltage The voltage to apply
   */
  public void setVoltage(double voltage) {
    this.targetAngleDegrees = Units.rotationsToDegrees(getPosition());
    this.targetVelocityDegPerSec = 0;

    if (!Constants.Turret.kTurretEnabled) {
      motor.setControl(positionRequest.withPosition(Constants.Turret.kMinAngleDegrees / 360.0));
      return;
    }

    motor.setVoltage(voltage);
  }

  /**
   * Get the pivot simulation for testing.
   * @return The pivot simulation model
   */
  public SingleJointedArmSim getSimulation() {
    return pivotSim;
  }

  /**
   * Creates a command to set the pivot to a specific angle.
   * @param angleDegrees The target angle in degrees
   * @return A command that sets the pivot to the specified angle
   */
  public Command setAngleCommand(double angleDegrees) {
    return runOnce(() -> setAngle(angleDegrees));
  }

  /**
   * Creates a command to move the pivot to a specific angle with a profile.
   * @param angleDegrees The target angle in degrees
   * @return A command that moves the pivot to the specified angle
   */
  public Command moveToAngleCommand(double angleDegrees) {
    return run(() -> {
      double currentAngle = Units.rotationsToDegrees(getPosition());
      double error = angleDegrees - currentAngle;
      double velocityDegPerSec =
        Math.signum(error) *
        Math.min(Math.abs(error) * 2.0, Units.radiansToDegrees(maxVelocity));
      setVelocity(velocityDegPerSec);
    })
      .until(() -> {
        double currentAngle = Units.rotationsToDegrees(getPosition());
        return Math.abs(angleDegrees - currentAngle) < 2.0; // 2 degree tolerance
      })
      .finallyDo(interrupted -> setVelocity(0));
  }

  /**
   * Creates a command to stop the pivot.
   * @return A command that stops the pivot
   */
  public Command stopCommand() {
    return runOnce(() -> setVelocity(0));
  }

  /**
   * Creates a command to move the pivot at a specific velocity.
   * @param velocityDegPerSec The target velocity in degrees per second
   * @return A command that moves the pivot at the specified velocity
   */
  public Command moveAtVelocityCommand(double velocityDegPerSec) {
    return run(() -> setVelocity(velocityDegPerSec));
  }

  /**
   * Creates a command to automatically aim the turret at a target.
   * @param robotPoseSupplier Supplier that provides the current robot pose
   * @param target The target to aim at (HUB, LEFT_PASS, or RIGHT_PASS)
   * @return A command that continuously aims the turret at the target
   */
  public Command autoAimCommand(Supplier<Pose2d> robotPoseSupplier, Supplier<ChassisSpeeds> fieldRelativeSpeedsSupplier, TurretUtil.TargetType target) {
    return run(() -> {
      Pose2d robotPose = robotPoseSupplier.get();
      ChassisSpeeds speeds = fieldRelativeSpeedsSupplier.get();
      double vx = speeds.vxMetersPerSecond;
      double vy = speeds.vyMetersPerSecond;

      TurretUtil.ShotSolution solution = TurretUtil.computeLeadShotSolution(robotPose, vx, vy, target);

      if (solution.isValid) {
        setAngle(solution.turretAngleDegrees);
      }
    }).finallyDo(__ -> setVelocity(0)).withName("AutoAim-" + target.toString());
  }

  //------------------------ Tuning -----------------------//

  /**
   * Sets turret angle and velocity using tunable PID values and setpoints from dashboard.
   * Updates PID gains in real-time and uses Setpoint1 for angle and Setpoint2 for velocity.
   * @param dashboard The dashboard publisher for retrieving tunable values
   */
  private void turretTunable() {
    // Get tunable PID values
    double kP = SmartDashboard.getNumber("Turret/kP", 0);
    double kI = SmartDashboard.getNumber("Turret/kI", 0);
    double kD = SmartDashboard.getNumber("Turret/kD", 0);
    double kV = SmartDashboard.getNumber("Turret/kV", 0);
    double kA = SmartDashboard.getNumber("Turret/kA", 0);
    double angleSetpoint =
    SmartDashboard.getNumber("Turret/AngleSetpoint", 0);
    double velocitySetpoint =
    SmartDashboard.getNumber("Turret/VelocitySetpoint", 0);


    // Update PID gains in slot 0
    Slot0Configs slot0 = new Slot0Configs();
    slot0.kP = kP;
    slot0.kI = kI;
    slot0.kD = kD;
    slot0.kV = kV;
    slot0.kA = kA;
    slot0.kS = kS; // Keep original kS
    
    motor.getConfigurator().apply(slot0);

    // Set angle or velocity based on which setpoint is non-zero
    // Priority: if angle setpoint is non-zero, use angle control
    // Otherwise, use velocity control if velocity setpoint is non-zero
    if (Math.abs(angleSetpoint) > 0.01) {
      setAngle(angleSetpoint);
    } else if (Math.abs(velocitySetpoint) > 0.01) {
      setVelocity(velocitySetpoint);
    } else {
      // Both are zero, stop the motor
      setVelocity(0);
    }
  }
  /**
   * Rezero the turret by re-applying the CANcoder magnet offset.
   * FusedCANcoder keeps the TalonFX synced automatically, so this just
   * reconfirms the offset in case it drifted after a hot restart.
   */
  private void rezero() {
    CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.MagnetOffset = Constants.Turret.kEncoderOffset;
    m_cancoder.getConfigurator().apply(cancoderConfig);
  }

  /**
   * Command to rezero the turret encoder.
   */
  public Command RezeroCommand() {
    return runOnce(() -> rezero());
  }

}