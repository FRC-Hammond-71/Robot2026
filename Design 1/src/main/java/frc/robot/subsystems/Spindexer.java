package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Spindexer extends SubsystemBase
{
    private final SparkMax motor = new SparkMax(40, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    private final SparkMax kicker = new SparkMax(52, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushed);

    public Spindexer() {
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.smartCurrentLimit(40);
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        SparkMaxConfig kickerConfig = new SparkMaxConfig();
        kickerConfig.smartCurrentLimit(40);
        kicker.configure(kickerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void clockwise(double speed) {
        motor.set(speed);
        kicker.set(-speed);
    }

    public void counterclockwise(double speed) {
        motor.set(-speed);
        kicker.set(speed);
    }

    public void stop() {
        motor.set(0);
        kicker.set(0);
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
