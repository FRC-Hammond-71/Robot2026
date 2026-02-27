package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Limelight.Limelight;

import java.util.Optional;

public class VisionSubsystem extends SubsystemBase {

    private static final String kLimelightName = "limelight";
    private final Limelight m_limelight;

    public VisionSubsystem() {
        Limelight.registerDevice(kLimelightName);
        m_limelight = Limelight.useDevice(kLimelightName);
    }

    /**
     * Returns stable filtered robot pose using Megatag2.
     * Requires current robot pose, gyro, and chassis speeds from drivetrain.
     */
    public Optional<Pose2d> getStablePose(Pose2d currentPose, Rotation2d gyro, ChassisSpeeds speeds) {
        return m_limelight.getStableEstimatedPose(currentPose, gyro, speeds);
    }

    public void resetPose(Pose2d initialPose) {
        m_limelight.resetPose(initialPose);
    }
}