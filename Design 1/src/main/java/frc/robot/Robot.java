// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;
    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run(); 
    }

    @Override
    public void disabledInit() {
        // m_robotContainer.cancelAllScheduled();
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    Command scheduledAuto;

    @Override
    public void autonomousInit() {
        // m_robotContainer.configBinds();

        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
            scheduledAuto = m_autonomousCommand;
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit()
    {
        if (scheduledAuto != null)
        {
            CommandScheduler.getInstance().cancel(scheduledAuto);
        }
    }


    @Override
    public void teleopInit() {
        // m_robotContainer.configBinds();
    }



    @Override
    public void teleopPeriodic() {

    }


    @Override
	public void robotInit() {
		System.out.println("Waiting for connection to Driver Station...");		
		while (!DriverStation.waitForDsConnection(2))
		{
			System.out.println("Retrying connection to Driver Station...");
		}
		System.out.println("Connected to Driver Station!");

		frc.robot.Limelight.Limelight.registerDevice("limelight");
    }


    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        // m_robotContainer.cancelAllScheduled();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
 