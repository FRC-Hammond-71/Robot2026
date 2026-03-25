package frc.robot.generated;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldConstants{

    /** Set to true to force Red alliance targeting when DS doesn't report one. */
    public static boolean forceRedAlliance = false;
    
    public static final double HUB_ENTRANCE_HEIGHT = 1.83;

    private static final Translation3d RED_HUB = new Translation3d(11.938, 4.034536, HUB_ENTRANCE_HEIGHT);
    public static final Translation3d RED_OUTPOST = new Translation3d(15.75, 7.25, 0);
    public static final Translation3d RED_FAR_SIDE = new Translation3d(15.75, 0.75, 0);
    private static final Translation3d BLUE_HUB = new Translation3d(4.5974, 4.034536, HUB_ENTRANCE_HEIGHT);
    public static final Translation3d BLUE_OUTPOST = new Translation3d(0.75, 0.75, 0);
    public static final Translation3d BLUE_FAR_SIDE = new Translation3d(0.75, 7.25, 0);


    public static final Pose3d leftPassTarget = new Pose3d(3.5, 6.0, 0, new Rotation3d());
    public static final Pose3d rightPassTarget = new Pose3d(3.5, 1.960, 0, new Rotation3d());


    // public static Translation3d getAllianceHub() {
    //     return DriverStation.getAlliance().get() == Alliance.Blue ? BLUE_HUB : RED_HUB;
    // }
    // DAY 2 FIX PRIORITY 2 - COMMENT OUT ABOVE METHOD AND UNCOMMENT BELOW
// DEFENSIVE FIX - prevents crash or wrong hub if DS connection slow at startup
    public static Translation3d getAllianceHub() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == Alliance.Blue ? BLUE_HUB : RED_HUB;
        }
        return forceRedAlliance ? RED_HUB : BLUE_HUB;
    }

    public static Translation3d getClosestHub(Translation2d robotPosition) {
        double distBlue = robotPosition.getDistance(BLUE_HUB.toTranslation2d());
        double distRed = robotPosition.getDistance(RED_HUB.toTranslation2d());
        return distBlue <= distRed ? BLUE_HUB : RED_HUB;
    }

    private static final StructArrayPublisher<Pose3d> hubPosesPub =
            NetworkTableInstance.getDefault()
                    .getStructArrayTopic("Field/HubEntrances", Pose3d.struct)
                    .publish();

    private static final Pose3d[] HUB_POSES = new Pose3d[] {
            new Pose3d(BLUE_HUB, new Rotation3d()),
            new Pose3d(RED_HUB, new Rotation3d())
    };

    public static void publishFieldPoses() {
        hubPosesPub.set(HUB_POSES);
    }

}
