package frc.robot.generated;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldConstants{
    
    public static final Translation3d RED_HUB = new Translation3d(11.938, 4.034536, 1.5748);
    public static final Translation3d RED_OUTPOST = new Translation3d(15.75, 7.25, 0);
    public static final Translation3d RED_FAR_SIDE = new Translation3d(15.75, 0.75, 0);
    public static final Translation3d BLUE_HUB = new Translation3d(4.5974, 4.034536, 1.5748);
    public static final Translation3d BLUE_OUTPOST = new Translation3d(0.75, 0.75, 0);
    public static final Translation3d BLUE_FAR_SIDE = new Translation3d(0.75, 7.25, 0);


    public static final Pose3d hubTarget = new Pose3d(4.620, 4.040, 3.057144, new Rotation3d());
    public static final Pose3d leftPassTarget = new Pose3d(2.50, 6.0, 0, new Rotation3d());
    public static final Pose3d rightPassTarget = new Pose3d(2.50, 1.960, 0, new Rotation3d());


    public static Translation3d getAllianceHub() {
        return DriverStation.getAlliance().get() == Alliance.Blue ? BLUE_HUB : RED_HUB;
    }

}