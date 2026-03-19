package frc.robot.Math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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
}
