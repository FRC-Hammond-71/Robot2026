package frc.robot.odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.MathUtil;

// Scales odometry deltas by cos(tilt) so bumps/ramps don't inflate distance.
public class TiltCompensatedOdometry {

    private static final double MAX_TILT_DEG = 25.0;

    private Pose2d lastRawPose = new Pose2d();
    private Pose2d compensatedPose = new Pose2d();

    public void reset(Pose2d pose) {
        lastRawPose = pose;
        compensatedPose = pose;
    }

    public Pose2d update(Pose2d rawPose, double pitchDeg, double rollDeg) {
        Transform2d delta = new Transform2d(lastRawPose, rawPose);

        double tiltDeg = Math.sqrt(pitchDeg * pitchDeg + rollDeg * rollDeg);
        double scale = 1.0 - MathUtil.inverseInterpolate(0.0, MAX_TILT_DEG, tiltDeg);
        scale = MathUtil.clamp(scale, 0.0, 1.0);

        // Only scale translation; rotation comes from gyro
        Translation2d scaledTranslation = delta.getTranslation().times(scale);
        Rotation2d deltaRotation = delta.getRotation();

        Twist2d twist = new Twist2d(
            scaledTranslation.getX(),
            scaledTranslation.getY(),
            deltaRotation.getRadians());
        compensatedPose = compensatedPose.exp(twist);

        lastRawPose = rawPose;
        return compensatedPose;
    }

    public Pose2d getCompensatedPose() {
        return compensatedPose;
    }

    // 1.0 = flat, 0.0 = max tilt
    public static double tiltFactor(double pitchDeg, double rollDeg) {
        double tiltDeg = Math.sqrt(pitchDeg * pitchDeg + rollDeg * rollDeg);
        double scale = 1.0 - MathUtil.inverseInterpolate(0.0, MAX_TILT_DEG, tiltDeg);
        return MathUtil.clamp(scale, 0.0, 1.0);
    }
}
