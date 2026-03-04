package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

    // Kraken — pulls robot up once engaged
    private final TalonFX m_climbMotor = new TalonFX(19);

    // Spark Max + brushed DC — linear extension to engage climbing structure
    private final SparkMax m_extensionMotor = new SparkMax(
        Constants.Climber.kExtensionSparkCanID, MotorType.kBrushed
    );

    public Climber() {
        // Kraken config — brake so robot stays up when command ends
        TalonFXConfiguration climbConfig = new TalonFXConfiguration();
        climbConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        climbConfig.CurrentLimits.StatorCurrentLimit = 80;
        climbConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        m_climbMotor.getConfigurator().apply(climbConfig);

        // Extension motor config
        SparkMaxConfig extensionConfig = new SparkMaxConfig();
        extensionConfig.smartCurrentLimit(40);
        m_extensionMotor.configure(extensionConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    // --- Climb (Kraken) ---

    public void climbUp(double speed) {
        m_climbMotor.set(-speed);
    }

    public void climbDown(double speed) {
        m_climbMotor.set(speed);
    }

    public void stopClimb() {
        m_climbMotor.set(0);
    }

    public Command upCommand(double speed) {
        return Commands.runEnd(() -> climbUp(speed), this::stopClimb, this).withName("Climber Up");
    }

    public Command downCommand(double speed) {
        return Commands.runEnd(() -> climbDown(speed), this::stopClimb, this).withName("Climber Down");
    }

    // --- Extension (Spark Max) ---

    public void extend(double speed) {
        m_extensionMotor.set(speed);
    }

    public void stopExtension() {
        m_extensionMotor.set(0);
    }

    public Command extendCommand() {
        return Commands.runEnd(
            () -> extend(Constants.Climber.kExtensionSpeedPercent),
            this::stopExtension,
            this
        ).withTimeout(Constants.Climber.kExtensionTimeSeconds).withName("Climber Extend");
    }

    public Command retractCommand() {
        return Commands.runEnd(
            () -> extend(-Constants.Climber.kExtensionSpeedPercent),
            this::stopExtension,
            this
        ).withTimeout(Constants.Climber.kExtensionTimeSeconds).withName("Climber Retract");
    }

    // --- Timed climb at constants speed ---

    public Command climbCommand() {
        return upCommand(Constants.Climber.kClimbSpeedPercent)
            .withTimeout(Constants.Climber.kClimbTimeSeconds)
            .withName("Climb");
    }
}
