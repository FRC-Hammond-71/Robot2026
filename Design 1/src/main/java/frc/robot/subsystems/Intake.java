package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Commands.CommandUtils;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class Intake extends SubsystemBase {

    private final TalonFX motor = new TalonFX(18);
    private final DutyCycleOut request = new DutyCycleOut(0.1);

    public Intake() {
        TalonFXConfiguration configs = new TalonFXConfiguration();
        motor.getConfigurator().apply(configs);
    }

    public void intake(double speed) {
        motor.set(-speed);
    }

    public void score(double speed) {
        motor.set(speed);
    }

    public void stop() {
        motor.set(0);
    }


    public Command IntakeCommand() { return CommandUtils.withName(("Intake"),intakeCommand(0.5));
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
