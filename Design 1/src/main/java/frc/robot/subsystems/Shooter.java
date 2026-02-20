package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Commands.CommandUtils;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class Shooter extends SubsystemBase {

    private final TalonFX m_intakeA = new TalonFX(41);
    private final TalonFX m_intakeB = new TalonFX(42);
    

    public Shooter() {
        TalonFXConfiguration configs = new TalonFXConfiguration();
        m_intakeA.getConfigurator().apply(configs);
        m_intakeB.getConfigurator().apply(configs);
    }

    public void intake(double speed) {
        m_intakeA.set(speed);
        m_intakeB.set(-speed);
    }

    public void score(double speed) {
        m_intakeA.set(-speed);
        m_intakeB.set(speed);
    }

    public void stop() {
        m_intakeA.set(0);
        m_intakeB.set(0);
    }

    public Command intakeCommand(double speed) {
        return new RunCommand(
            () -> intake(speed),
            this
        ).finallyDo(this::stop);
    }

    public Command scoreCommand(double speed) {
        return new RunCommand(
            () -> score(speed),
            this
        ).finallyDo(this::stop);
    }
}
