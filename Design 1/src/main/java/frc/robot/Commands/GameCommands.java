package frc.robot.Commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;

public class GameCommands 
{
    public final Command Intake() 
    { 
        return CommandUtils.withName("Intake", r.intake.IntakeCommand());
    }

    private final Robot r;

    public GameCommands(Robot r)
    {
        this.r = r;
    }    
}
