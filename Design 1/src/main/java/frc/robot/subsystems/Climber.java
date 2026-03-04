package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

    // ─── Hardware ──────────────────────────────────────────────────────────

    /** Kraken X60 (TalonFX) — pulls the robot up. */
    private final TalonFX climbMotor = new TalonFX(Constants.Climber.kClimberCanID);

    /** Brushed DC motor — extends the climber arm before latching. */
    private final PWMSparkMax extensionMotor = new PWMSparkMax(Constants.Climber.kExtensionMotorPort);

    /** Reusable swerve request for the auto-aim heading lock. */
    private final SwerveRequest.FieldCentric autoAimRequest = new SwerveRequest.FieldCentric();

    // ──────────────────────────────────────────────────────────────────────

    public Climber() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Brake mode keeps the robot held up when the motor is idle
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // PID + feedforward for the Kraken X60 (slot 0)
        Slot0Configs slot0 = config.Slot0;
        slot0.kP = Constants.Climber.kP;
        slot0.kI = Constants.Climber.kI;
        slot0.kD = Constants.Climber.kD;
        slot0.kS = Constants.Climber.kS;
        slot0.kV = Constants.Climber.kV;

        // Gear ratio so the controller works in mechanism rotations
        config.Feedback.SensorToMechanismRatio = Constants.Climber.kGearRatio;

        // Protect the motor from stall current spikes
        CurrentLimitsConfigs limits = config.CurrentLimits;
        limits.StatorCurrentLimit = 40;
        limits.StatorCurrentLimitEnable = true;

        climbMotor.getConfigurator().apply(config);
        climbMotor.setPosition(0);
    }

    // ─── Kraken X60 low-level methods ─────────────────────────────────────

    /** Drives the climb motor upward at the given duty-cycle (0–1). */
    public void up(double speed) {
        climbMotor.set(-speed);
    }

    /** Drives the climb motor downward at the given duty-cycle (0–1). */
    public void down(double speed) {
        climbMotor.set(speed);
    }

    /** Stops the Kraken X60. */
    public void stop() {
        climbMotor.set(0);
    }

    // ─── Extension DC motor low-level methods ─────────────────────────────

    /** Extends the arm at the given duty-cycle (0–1). */
    public void extend(double speed) {
        extensionMotor.set(speed);
    }

    /** Retracts the arm at the given duty-cycle (0–1). */
    public void retract(double speed) {
        extensionMotor.set(-speed);
    }

    /** Stops the extension motor. */
    public void stopExtension() {
        extensionMotor.set(0);
    }

    // ─── Commands ─────────────────────────────────────────────────────────

    /** Runs the Kraken X60 upward while active; stops on end. */
    public Command upCommand(double speed) {
        return this.runEnd(() -> up(speed), this::stop).withName("Climber Up");
    }

    /** Runs the Kraken X60 downward while active; stops on end. */
    public Command downCommand(double speed) {
        return this.runEnd(() -> down(speed), this::stop).withName("Climber Down");
    }

    /**
     * Runs the extension DC motor for {@link Constants.Climber#kExtensionTimeSeconds}
     * at {@link Constants.Climber#kExtensionSpeed}, then stops automatically.
     * Safe to use in both teleop and autonomous.
     */
    public Command extendCommand() {
        return this.runEnd(
                () -> extend(Constants.Climber.kExtensionSpeed),
                this::stopExtension)
            .withTimeout(Constants.Climber.kExtensionTimeSeconds)
            .withName("Climber Extend");
    }

    /**
     * Runs the Kraken X60 upward for {@link Constants.Climber#TimeInSeconds}
     * at {@link Constants.Climber#SpeedPercent}, then stops.
     * Use during competition to complete the climb.
     */
    public Command climbCommand() {
        return upCommand(Constants.Climber.SpeedPercent)
            .withTimeout(Constants.Climber.TimeInSeconds)
            .withName("Climb");
    }

    /**
     * Full climb sequence: extend the arm via the DC motor, then pull the robot
     * up with the Kraken X60. Safe to schedule in autonomous routines.
     */
    public Command fullClimbSequenceCommand() {
        return Commands.sequence(
            extendCommand(),
            climbCommand()
        ).withName("Full Climb Sequence");
    }

    /**
     * Rotates the robot to face the climb target defined in
     * {@link Constants.Climber#kClimbTargetX}/{@link Constants.Climber#kClimbTargetY}
     * using a proportional heading controller, then runs the full climb sequence.
     *
     * <p>Suitable for use inside an autonomous command group.
     *
     * @param drivetrain the swerve drivetrain used for rotation
     * @return a command that auto-aims then climbs
     */
    public Command autoAimAndClimbCommand(CommandSwerveDrivetrain drivetrain) {
        return Commands.sequence(
            autoAimCommand(drivetrain),
            fullClimbSequenceCommand()
        ).withName("Auto-Aim and Climb");
    }

    /**
     * Rotates the robot to face the climb target using odometry feedback.
     * Ends when the heading error is within
     * {@link Constants.Climber#kHeadingToleranceDegrees} or after 3 seconds.
     *
     * @param drivetrain the swerve drivetrain used for rotation
     * @return a command that aligns the robot heading toward the climb target
     */
    public Command autoAimCommand(CommandSwerveDrivetrain drivetrain) {
        return drivetrain.applyRequest(() -> {
            Pose2d pose = drivetrain.getState().Pose;
            double dx = Constants.Climber.kClimbTargetX - pose.getX();
            double dy = Constants.Climber.kClimbTargetY - pose.getY();
            double targetAngle = Math.atan2(dy, dx);
            double currentAngle = pose.getRotation().getRadians();

            // Normalize error to [-π, π] to always take the shortest path
            double error = Math.atan2(
                Math.sin(targetAngle - currentAngle),
                Math.cos(targetAngle - currentAngle)
            );

            double rotRate = error * Constants.Climber.kHeadingKP;

            return autoAimRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(rotRate);
        })
        .until(() -> {
            Pose2d pose = drivetrain.getState().Pose;
            double dx = Constants.Climber.kClimbTargetX - pose.getX();
            double dy = Constants.Climber.kClimbTargetY - pose.getY();
            double targetAngle = Math.atan2(dy, dx);
            double currentAngle = pose.getRotation().getRadians();
            double error = Math.atan2(
                Math.sin(targetAngle - currentAngle),
                Math.cos(targetAngle - currentAngle)
            );
            return Math.abs(error) < Math.toRadians(Constants.Climber.kHeadingToleranceDegrees);
        })
        .withTimeout(3.0)
        .withName("Climber Auto-Aim");
    }
}
