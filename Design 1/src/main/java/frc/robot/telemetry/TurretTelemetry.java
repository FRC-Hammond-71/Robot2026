package frc.robot.telemetry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.Constants;
import frc.robot.generated.FieldConstants;
import frc.robot.util.dashboard.TurretUtil;

public class TurretTelemetry {
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable turretTable = inst.getTable("Turret");

    private final StructPublisher<Pose2d> turretFieldPose = turretTable.getStructTopic("FieldPose", Pose2d.struct).publish();
    private final StructPublisher<Pose2d> leadShotFieldPose = turretTable.getStructTopic("LeadShotPose", Pose2d.struct).publish();
    private final StructPublisher<Pose3d> launchOriginPose = inst.getStructTopic("Field/LaunchOrigin", Pose3d.struct).publish();
    private final StructArrayPublisher<Translation2d> turretToHubPub = turretTable.getStructArrayTopic("TurretToHub", Translation2d.struct).publish();

    public void publish(Pose2d robotPose, Rotation2d turretFieldHeading, Pose2d leadShotPose) {
        Translation2d hub = FieldConstants.getAllianceHub().toTranslation2d();
        Translation2d turretTranslation = TurretUtil.getTurretPose(robotPose).getTranslation();

        Pose2d turretPose = new Pose2d(turretTranslation, turretFieldHeading);
        turretFieldPose.set(turretPose);

        turretToHubPub.set(new Translation2d[] { turretTranslation, hub });
        leadShotFieldPose.set(leadShotPose);

        /* Publish 3D launch origin: turret XY + turret Z height */
        Pose2d turret2d = TurretUtil.getTurretPose(robotPose);
        launchOriginPose.set(new Pose3d(
                turret2d.getX(),
                turret2d.getY(),
                Constants.Turret.kTurretOffsetZ,
                new Rotation3d(0, 0, turretFieldHeading.getRadians())));
    }

    /** Returns the turret translation for use by other telemetry classes. */
    public Translation2d getTurretTranslation(Pose2d robotPose) {
        return TurretUtil.getTurretPose(robotPose).getTranslation();
    }
}
