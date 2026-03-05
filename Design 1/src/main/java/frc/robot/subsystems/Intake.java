package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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

public class Intake extends SubsystemBase {

    // KrakenX60 — intake shaft
    private final TalonFX m_intakeMotor = new TalonFX(Constants.Intake.kIntakeMotorCanID);

    // NEO — linear extension (14t driving 48t)
    private final SparkMax m_extensionMotor = new SparkMax(Constants.Intake.kExtensionMotorCanID, MotorType.kBrushless);

    public Intake() {
        // KrakenX60 config
        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
        intakeConfig.CurrentLimits.StatorCurrentLimit = Constants.Intake.kStatorCurrentLimit;
        intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        m_intakeMotor.getConfigurator().apply(intakeConfig);

        // NEO extension config — gear ratio applied as position conversion factor
        SparkMaxConfig extensionConfig = new SparkMaxConfig();
        extensionConfig.smartCurrentLimit(Constants.Intake.kExtensionCurrentLimit);
        extensionConfig.encoder.positionConversionFactor(1.0 / Constants.Intake.kExtensionGearRatio);
        m_extensionMotor.configure(extensionConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    // --- Intake shaft (KrakenX60) ---

    public void intake(double speed) {
        m_intakeMotor.set(-speed);
    }

    public void score(double speed) {
        m_intakeMotor.set(speed);
    }

    public void stop() {
        m_intakeMotor.set(0);
    }

    public Command intakeCommand(double speed) {
        return Commands.runEnd(() -> intake(speed), this::stop, this).withName("Intake");
    }

    public Command scoreCommand(double speed) {
        return Commands.runEnd(() -> score(speed), this::stop, this).withName("Score");
    }

    public void extend(double speed) {
        m_extensionMotor.set(speed);
    }

    public void retract(double speed) {
        m_extensionMotor.set(-speed);
    }

    public void stopExtension() {
        m_extensionMotor.set(0);
    }

    public double getExtensionPosition() {
        return m_extensionMotor.getEncoder().getPosition(); // output shaft rotations
    }

    public Command extendCommand(double speed) {
        return Commands.runEnd(() -> extend(speed), this::stopExtension, this).withName("Intake Extend");
    }

    public Command retractCommand(double speed) {
        return Commands.runEnd(() -> retract(speed), this::stopExtension, this).withName("Intake Retract");
    }
}
