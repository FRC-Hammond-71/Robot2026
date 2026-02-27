package frc.robot.Commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;

public class ShootFuelCommand extends Command {

    private final Shooter m_shooter;
    private final CommandSwerveDrivetrain m_drivetrain;
    private double m_targetRPS = 0;

    public ShootFuelCommand(Shooter shooter, CommandSwerveDrivetrain drivetrain) {
        m_shooter    = shooter;
        m_drivetrain = drivetrain;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        Pose2d currentPose = m_drivetrain.getState().Pose;
        double distance    = m_shooter.calculateDistance(currentPose.getTranslation());
        m_targetRPS        = m_shooter.calculateTargetRPS(distance);
        m_shooter.setVelocity(m_targetRPS);

        SmartDashboard.putNumber("Shooter/TargetRPS",      m_targetRPS);
        SmartDashboard.putNumber("Shooter/DistanceMeters", distance);
        SmartDashboard.putBoolean("Shooter/Ready",         m_shooter.isAtSpeed(m_targetRPS));
    }

    @Override
    public boolean isFinished() {
        return false; // whileTrue handles termination on button release
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stop();
    }
}