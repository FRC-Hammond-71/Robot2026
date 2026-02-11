package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Spindexer extends SubsystemBase{
    private final SparkMax motor = new SparkMax(40, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushed);



    public void clockwise(double speed) {
        motor.set(speed);
    }

    public void counterclockwise(double speed) {
        motor.set(-speed);
    }

    public void stop() {
        motor.set(0);
    }

    public Command clockwiseCommand(double speed) {
        return new RunCommand(
            () -> clockwise(speed),
            this
        ).finallyDo(this::stop);
    }

    public Command counterClockwiseCommand(double speed) {
        return new RunCommand(
            () -> counterclockwise(speed),
            this
        ).finallyDo(this::stop);
    }


}
