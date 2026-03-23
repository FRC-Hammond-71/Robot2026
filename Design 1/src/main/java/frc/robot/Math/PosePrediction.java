package frc.robot.Math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class PosePrediction
{
    public static Pose2d Linear(Pose2d inital, ChassisSpeeds velocity, double timeInSeconds)
    {
        var transformOverTime = new Transform2d(
            velocity.vxMetersPerSecond * timeInSeconds,
            velocity.vyMetersPerSecond * timeInSeconds,
            Rotation2d.fromRadians(velocity.omegaRadiansPerSecond * timeInSeconds)
        );

        return inital.plus(transformOverTime);
    }

    // Like Linear but integrates along the arc — way more accurate when rotating.
    public static Pose2d Exponential(Pose2d initial, ChassisSpeeds velocity, double timeInSeconds) {
        Twist2d twist = new Twist2d(
            velocity.vxMetersPerSecond * timeInSeconds,
            velocity.vyMetersPerSecond * timeInSeconds,
            velocity.omegaRadiansPerSecond * timeInSeconds);
        return initial.exp(twist);
    }
}
