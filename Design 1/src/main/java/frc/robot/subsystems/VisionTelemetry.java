package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.Constants;

/**
 * Publishes vision debug telemetry to NetworkTables.
 * In competition mode only essential quality metrics are published.
 * In debug mode, full transform-chain and pose overlays are included.
 */
public final class VisionTelemetry {

    private static final Pose3d[] EMPTY_POSE3D_ARRAY = new Pose3d[0];
    private final boolean competitionMode;

    // --- Essential (always published) ---
    private final StructPublisher<Pose2d> robotPoseEstimate;
    private final DoublePublisher correctionDelta;
    private final DoublePublisher tagCount;
    private final DoublePublisher yawRate;

    private final StructPublisher<Pose2d> turretFieldPose;
    private final StructPublisher<Pose2d> cameraFieldPose;

    // --- Debug only (null in competition mode) ---
    private final StructPublisher<Pose2d> cameraPose;
    private final StructPublisher<Pose2d> rawPose;
    private final StructPublisher<Pose2d> megaTagPose;
    private final StructArrayPublisher<Pose3d> visibleTagPoses;
    private final DoublePublisher turretAngleDeg;
    private final DoublePublisher pigeonHeadingDeg;
    private final DoublePublisher cameraHeadingDeg;
    private final DoublePublisher visionTimestamp;

    public VisionTelemetry(boolean competitionMode) {
        this.competitionMode = competitionMode;
        var nt = NetworkTableInstance.getDefault();
        var table = nt.getTable("Vision");

        // Essential publishers
        robotPoseEstimate = table.getStructTopic("RobotPoseEstimate", Pose2d.struct).publish();
        correctionDelta = table.getDoubleTopic("CorrectionDeltaM").publish();
        tagCount = table.getDoubleTopic("TagCount").publish();
        yawRate = table.getDoubleTopic("YawRateDegPerSec").publish();

        // Always-on field poses
        turretFieldPose = table.getStructTopic("TurretFieldPose", Pose2d.struct).publish();
        cameraFieldPose = table.getStructTopic("CameraFieldPose", Pose2d.struct).publish();

        // Debug publishers — only allocate if not in competition mode
        if (!competitionMode) {
            var turretTable = table.getSubTable("Turret");
            cameraPose = table.getStructTopic("CameraPose", Pose2d.struct).publish();
            rawPose = table.getStructTopic("RawPose", Pose2d.struct).publish();
            megaTagPose = table.getStructTopic("MegaTagPose", Pose2d.struct).publish();
            visibleTagPoses = table.getStructArrayTopic("VisibleTagPoses", Pose3d.struct).publish();
            turretAngleDeg = turretTable.getDoubleTopic("AngleDeg").publish();
            pigeonHeadingDeg = turretTable.getDoubleTopic("PigeonHeadingDeg").publish();
            cameraHeadingDeg = turretTable.getDoubleTopic("CameraHeadingDeg").publish();
            visionTimestamp = table.getDoubleTopic("Timestamp").publish();
        } else {
            cameraPose = null;
            rawPose = null;
            megaTagPose = null;
            visibleTagPoses = null;
            turretAngleDeg = null;
            pigeonHeadingDeg = null;
            cameraHeadingDeg = null;
            visionTimestamp = null;
        }
    }

    public void publishPoses(Pose2d robotPose, double turretAngleRad) {
        double robotHeading = robotPose.getRotation().getRadians();
        double cosR = Math.cos(robotHeading);
        double sinR = Math.sin(robotHeading);

        double tx = Constants.Turret.kTurretOffsetX;
        double ty = Constants.Turret.kTurretOffsetY;
        double turretFieldX = robotPose.getX() + cosR * tx - sinR * ty;
        double turretFieldY = robotPose.getY() + sinR * tx + cosR * ty;
        double turretHeading = robotHeading + turretAngleRad;
        turretFieldPose.set(new Pose2d(turretFieldX, turretFieldY, new Rotation2d(turretHeading)));

        double cosT = Math.cos(turretHeading);
        double sinT = Math.sin(turretHeading);
        double cx = Constants.Vision.kCamFromTurretX;
        double cy = Constants.Vision.kCamFromTurretY;
        double camFieldX = turretFieldX + cosT * cx - sinT * cy;
        double camFieldY = turretFieldY + sinT * cx + cosT * cy;
        cameraFieldPose.set(new Pose2d(camFieldX, camFieldY, new Rotation2d(turretHeading)));
    }

    /** Publish all telemetry from a successful vision result. */
    public void publish(VisionSubsystem.VisionResult result) {
        var t = result.telemetry();
        var vm = result.measurement();

        // Essential
        robotPoseEstimate.set(vm.pose());
        correctionDelta.set(t.correctionDeltaM());
        tagCount.set(t.tagCount());
        yawRate.set(t.yawRateDegPerSec());

        // Debug
        if (!competitionMode) {
            cameraPose.set(t.correctedCameraPose());
            megaTagPose.set(vm.pose());
            result.rawPose().ifPresent(p -> rawPose.set(p));
            visibleTagPoses.set(result.visibleTagPoses());
            turretAngleDeg.set(t.turretAngleDeg());
            pigeonHeadingDeg.set(t.pigeonHeadingDeg());
            cameraHeadingDeg.set(t.cameraHeadingDeg());
            visionTimestamp.set(t.visionTimestamp());
        }
    }
}
