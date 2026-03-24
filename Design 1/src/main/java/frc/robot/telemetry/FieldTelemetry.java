package frc.robot.telemetry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generated.FieldConstants;
import frc.robot.util.dashboard.TurretUtil;

public class FieldTelemetry {
    public void publish(Pose2d robotPose) {
        Translation2d hub = FieldConstants.getAllianceHub().toTranslation2d();
        SmartDashboard.putNumber("DistanceToHub/Robot", robotPose.getTranslation().getDistance(hub));
        SmartDashboard.putNumber("DistanceToHub/Turret", TurretUtil.getTurretPose(robotPose).getTranslation().getDistance(hub));
    }
}
