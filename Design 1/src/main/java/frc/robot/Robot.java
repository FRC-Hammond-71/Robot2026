// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.ctre.phoenix6.Timestamp.TimestampSource;
import com.ctre.phoenix6.Utils;

import org.ironmaple.simulation.SimulatedArena;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionTelemetry;

public class Robot extends TimedRobot {

    public final CommandSwerveDrivetrain Drivetrain;
    public final IntakeSubsystem Intake;
    public final ShooterSubsystem Shooter;
    public final SpindexerSubsystem Spindexer;
    public final TurretSubsystem Turret;
    public VisionSubsystem Vision;

    private final VisionTelemetry visionTelemetry;

    public Robot() {
        Turret = new TurretSubsystem(this);
        Drivetrain = new CommandSwerveDrivetrain(
            TunerConstants.DrivetrainConstants,
            100,
            TunerConstants.FrontLeft, TunerConstants.FrontRight,
            TunerConstants.BackLeft, TunerConstants.BackRight);

        visionTelemetry = new VisionTelemetry();

        try {
            Vision = new VisionSubsystem();
        } catch (Exception ex) {
            System.err.println("Warning: VisionSubsystem could not be constructed: "
                + ex.getMessage());
            Vision = null;
        }

        Drivetrain.setBufferedTurretAngle(Turret.getPositionSignal());

        Intake = new IntakeSubsystem(this);
        Spindexer = new SpindexerSubsystem();
        Shooter = new ShooterSubsystem(this);

        new RobotContainer(this);
    }

    @Override
    public void robotInit() {
        System.out.println("Waiting on Driver Station...");
        while (!DriverStation.waitForDsConnection(2)) {}
        System.out.println("Connected to Driver Station!");

        frc.robot.Limelight.Limelight.registerDevice("limelight");
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        // Always-on turret/camera field poses (every frame, independent of vision)
        double turretAngleRad = VisionSubsystem.turretRotationsToRadians(Turret.getPosition());
        visionTelemetry.publishPoses(Drivetrain.getState().Pose, turretAngleRad);

        try {
            if (Vision != null) {
                var measOpt = getVisionMeasurement();

                if (measOpt.isPresent()) {
                    var result = measOpt.get();
                    visionTelemetry.publish(result);

                    var vm = result.measurement();
                    Drivetrain.addVisionMeasurement(
                        vm.pose(), vm.timestampSeconds(),
                        VecBuilder.fill(0.5, 0.5, 1e9));
                }
            }
        } catch (Exception ex) {
            ex.printStackTrace();
        }
    }

    private Optional<VisionSubsystem.VisionResult> getVisionMeasurement() {
        var buffered = Drivetrain.getBufferedTurretAngle();
        double pigeonYaw = Drivetrain.getPigeon2().getYaw().getValueAsDouble();
        Pose2d odom = Drivetrain.getState().Pose;
        double yawRate = Drivetrain.getPigeon2()
            .getAngularVelocityZWorld().getValueAsDouble();

        if (Utils.isSimulation()) {
            return Vision.processSimVision(
                buffered, pigeonYaw, odom, yawRate);
        }

        var limelightOpt = frc.robot.Limelight.Limelight.useDevice("limelight");
        if (limelightOpt.isEmpty()) {
            return Optional.empty();
        }
        return Vision.processLimelight(
            limelightOpt.get(), buffered, pigeonYaw, odom, yawRate);
    }

    // ---- Autonomous ----

    public Command selectedAutoCommand;
    private Command scheduledAuto;

    @Override
    public void autonomousInit() {
        scheduledAuto = selectedAutoCommand;
        if (selectedAutoCommand != null) {
            CommandScheduler.getInstance().schedule(selectedAutoCommand);
        }
    }

    @Override
    public void autonomousExit() {
        if (scheduledAuto != null) {
            CommandScheduler.getInstance().cancel(scheduledAuto);
        }
    }

    // ---- Teleop ----

    @Override
    public void teleopInit() {}

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    // ---- Test ----

    @Override
    public void testInit() {}

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    // ---- Simulation ----

    private final StructArrayPublisher<Pose3d> gamePiecePub =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("Sim/GamePieces", Pose3d.struct).publish();

    private final StructPublisher<Pose2d> simRPosePub =
        NetworkTableInstance.getDefault()
            .getStructTopic("Sim/rPose", Pose2d.struct).publish();

    @Override
    public void simulationInit() {
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    @Override
    public void simulationPeriodic() {
        SimulatedArena.getInstance().simulationPeriodic();

        gamePiecePub.set(
            SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
        simRPosePub.set(
            Drivetrain.getMapleSimSwerveDrivetrain().getSimulatedDriveTrainPose());

        // Update vision sim camera transform for turret rotation
        try {
            if (Vision != null) {
                Pose2d simPose = Drivetrain.getMapleSimSwerveDrivetrain()
                    .getSimulatedDriveTrainPose();
                double turretAngleRad = 0.0;
                var buffered = Drivetrain.getBufferedTurretAngle();
                if (buffered != null) {
                    double turretRot = buffered.getValueAt(
                        Timer.getFPGATimestamp(), TimestampSource.System);
                    turretAngleRad = VisionSubsystem.turretRotationsToRadians(turretRot);
                }
                Vision.adjustCameraForTurret(simPose, turretAngleRad);
                Vision.simUpdate(simPose);
            }
        } catch (Exception ex) {
            ex.printStackTrace();
        }
    }
}
