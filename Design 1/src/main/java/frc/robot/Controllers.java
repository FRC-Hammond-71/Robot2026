package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Controllers
{
	public static final CommandXboxController Joystick = new CommandXboxController(0);
	public static final CommandXboxController Operator = new CommandXboxController(RobotBase.isSimulation() ? 0 : 1);
}
