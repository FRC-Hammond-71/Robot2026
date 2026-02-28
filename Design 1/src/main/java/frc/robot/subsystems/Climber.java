package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Commands.CommandUtils;

public class Climber extends SubsystemBase {
    
    private final TalonFX motor = new TalonFX(19);
    // private final DutyCycleOut request = new DutyCycleOut(0.1);

    public Climber() {
        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motor.getConfigurator().apply(configs);
    }

    public void up(double speed) {
        motor.set(-speed);
    }

    public void down(double speed) {
        motor.set(speed);
    }

    public void stop() {
        motor.set(0);
    }

    public Command upCommand(double speed) 
    {
        return Commands.runEnd(() -> this.up(speed), this::stop).withName("Climber Up");
    }

    public Command downCommand(double speed) 
    {
        return Commands.runEnd(() -> this.down(speed), this::stop).withName("Climber Down");
    }

    /**
     * Use during competition to climb, then stop after a certain amount of time.
     */
    public Command climbCommand()
    {
        // Climb up at Constants.Climber.SpeedPercent and stop after Constants.Climber.TimeInSeconds.
        return upCommand(Constants.Climber.SpeedPercent).withTimeout(Constants.Climber.TimeInSeconds).withName("Climb");
    }
}