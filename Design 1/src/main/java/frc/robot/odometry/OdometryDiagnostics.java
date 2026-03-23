package frc.robot.odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Publishes wheel-only vs vision-fused pose so you can see how much vision is correcting.
public class OdometryDiagnostics {

    private static final String PREFIX = "Odometry/";

    public void publish(Pose2d wheelOnlyPose, Pose2d estimatedPose,
                        double pitchDeg, double rollDeg) {

        double correctionDelta = wheelOnlyPose.getTranslation().getDistance(estimatedPose.getTranslation());

        double headingDiffDeg = estimatedPose.getRotation().getDegrees() - wheelOnlyPose.getRotation().getDegrees();
        headingDiffDeg = ((headingDiffDeg + 180) % 360 + 360) % 360 - 180;

        SmartDashboard.putNumber(PREFIX + "WheelOnly/X", wheelOnlyPose.getX());
        SmartDashboard.putNumber(PREFIX + "WheelOnly/Y", wheelOnlyPose.getY());
        SmartDashboard.putNumber(PREFIX + "WheelOnly/HeadingDeg", wheelOnlyPose.getRotation().getDegrees());

        SmartDashboard.putNumber(PREFIX + "Estimated/X", estimatedPose.getX());
        SmartDashboard.putNumber(PREFIX + "Estimated/Y", estimatedPose.getY());
        SmartDashboard.putNumber(PREFIX + "Estimated/HeadingDeg", estimatedPose.getRotation().getDegrees());

        SmartDashboard.putNumber(PREFIX + "VisionCorrectionDeltaM", correctionDelta);
        SmartDashboard.putNumber(PREFIX + "HeadingDiffDeg", headingDiffDeg);
        SmartDashboard.putNumber(PREFIX + "PitchDeg", pitchDeg);
        SmartDashboard.putNumber(PREFIX + "RollDeg", rollDeg);
    }
}
