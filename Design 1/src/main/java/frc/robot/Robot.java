// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.ironmaple.simulation.SimulatedArena;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import com.ctre.phoenix6.Utils;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class Robot extends TimedRobot {

    public final CommandSwerveDrivetrain Drivetrain;

    public final IntakeSubsystem Intake;
    public final ShooterSubsystem Shooter;
    public final SpindexerSubsystem Spindexer;
    public final TurretSubsystem Turret;

    public Robot() {

        Turret = new TurretSubsystem(this);
        Drivetrain = new CommandSwerveDrivetrain(
                TunerConstants.DrivetrainConstants,
                100,
                TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight);
        Drivetrain.setBufferedTurretAngle(Turret.getPositionSignal());

        Intake = new IntakeSubsystem(this);
        Spindexer = new SpindexerSubsystem();
        Shooter = new ShooterSubsystem(this);

        new RobotContainer(this);

    }

    @Override
    public void robotInit() {
        System.out.println("Waiting on Driver Station...");

        while (!DriverStation.waitForDsConnection(2)) {
        }
        ;

        System.out.println("Connected to Driver Station!");

        frc.robot.Limelight.Limelight.registerDevice("limelight");
    }

    @Override
    public void robotPeriodic() {

        CommandScheduler.getInstance().run();

    }

    public Command selectedAutoCommand;
    private Command scheduledAuto;

    @Override
    public void autonomousInit() {

        scheduledAuto = selectedAutoCommand;

        if (selectedAutoCommand != null) {
            CommandScheduler.getInstance().schedule(selectedAutoCommand);
            scheduledAuto = selectedAutoCommand;
        }

    }

    @Override
    public void autonomousExit() {

        if (scheduledAuto != null) {
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
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        // m_robotContainer.cancelAllScheduled();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }

    private final StructArrayPublisher<Pose3d> gamePiecePub = NetworkTableInstance.getDefault()
            .getStructArrayTopic("Sim/GamePieces", Pose3d.struct).publish();

    private final StructPublisher<Pose2d> simRPosePub = NetworkTableInstance.getDefault()
            .getStructTopic("Sim/rPose", Pose2d.struct).publish();

    @Override
    public void simulationInit() {
        
        SimulatedArena.getInstance().resetFieldForAuto();

    }

    @Override
    public void simulationPeriodic() {

        SimulatedArena.getInstance().simulationPeriodic();

        gamePiecePub.set(SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));

        simRPosePub.set(Drivetrain.getMapleSimSwerveDrivetrain().getSimulatedDriveTrainPose());

    }
}
