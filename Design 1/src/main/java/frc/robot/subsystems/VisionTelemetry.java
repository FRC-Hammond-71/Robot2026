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
 * Publishes all vision debug telemetry to NetworkTables, structured for
 * AdvantageScope Field2d overlays and line-graph plotting.
 *
 * <p>Usage: call {@link #publish(VisionSubsystem.VisionResult)} when a
 * measurement is available. Call {@link #publishNoTargets()} only when
 * you know no tags are visible (not every frame — that causes flickering).
 */
public final class VisionTelemetry {

    private static final Pose3d[] EMPTY_POSE3D_ARRAY = new Pose3d[0];

    // --- Pose overlays (show on Field2d in AdvantageScope) ---
    private final StructPublisher<Pose2d> cameraPose;
    private final StructPublisher<Pose2d> robotPoseEstimate;
    private final StructPublisher<Pose2d> rawPose;
    private final StructPublisher<Pose2d> megaTagPose;

    // --- Tag visualization (3D so AdvantageScope can render at correct height) ---
    private final StructArrayPublisher<Pose3d> visibleTagPoses;

    // --- Turret transform chain (line graphs) ---
    private final DoublePublisher turretAngleDeg;
    private final DoublePublisher pigeonHeadingDeg;
    private final DoublePublisher cameraHeadingDeg;

    // --- Always-on field poses (published every frame) ---
    private final StructPublisher<Pose2d> turretFieldPose;
    private final StructPublisher<Pose2d> cameraFieldPose;

    // --- Quality metrics (line graphs) ---
    private final DoublePublisher correctionDelta;
    private final DoublePublisher tagCount;
    private final DoublePublisher yawRate;
    private final DoublePublisher visionTimestamp;

    public VisionTelemetry() {
        var nt = NetworkTableInstance.getDefault();
        var table = nt.getTable("Vision");
        var turretTable = table.getSubTable("Turret");

        cameraPose = table.getStructTopic("CameraPose", Pose2d.struct).publish();
        robotPoseEstimate = table.getStructTopic("RobotPoseEstimate", Pose2d.struct).publish();
        rawPose = table.getStructTopic("RawPose", Pose2d.struct).publish();
        megaTagPose = table.getStructTopic("MegaTagPose", Pose2d.struct).publish();

        visibleTagPoses = table.getStructArrayTopic("VisibleTagPoses", Pose3d.struct).publish();

        turretAngleDeg = turretTable.getDoubleTopic("AngleDeg").publish();
        pigeonHeadingDeg = turretTable.getDoubleTopic("PigeonHeadingDeg").publish();
        cameraHeadingDeg = turretTable.getDoubleTopic("CameraHeadingDeg").publish();

        turretFieldPose = table.getStructTopic("TurretFieldPose", Pose2d.struct).publish();
        cameraFieldPose = table.getStructTopic("CameraFieldPose", Pose2d.struct).publish();

        correctionDelta = table.getDoubleTopic("CorrectionDeltaM").publish();
        tagCount = table.getDoubleTopic("TagCount").publish();
        yawRate = table.getDoubleTopic("YawRateDegPerSec").publish();
        visionTimestamp = table.getDoubleTopic("Timestamp").publish();
    }

    /** Publish all telemetry from a successful vision result. */
    public void publish(VisionSubsystem.VisionResult result) {
        var t = result.telemetry();
        var vm = result.measurement();

        // Pose overlays
        cameraPose.set(t.correctedCameraPose());
        robotPoseEstimate.set(vm.pose());
        megaTagPose.set(vm.pose());
        result.rawPose().ifPresent(p -> rawPose.set(p));

        // Tag visualization
        visibleTagPoses.set(result.visibleTagPoses());

        // Turret transform chain
        turretAngleDeg.set(t.turretAngleDeg());
        pigeonHeadingDeg.set(t.pigeonHeadingDeg());
        cameraHeadingDeg.set(t.cameraHeadingDeg());

        // Quality metrics
        correctionDelta.set(t.correctionDeltaM());
        tagCount.set(t.tagCount());
        yawRate.set(t.yawRateDegPerSec());
        visionTimestamp.set(t.visionTimestamp());
    }

    /** Publish turret and camera field poses every frame, independent of vision results. */
    public void publishPoses(Pose2d robotPose, double turretAngleRad) {
        double robotHeading = robotPose.getRotation().getRadians();
        double cosR = Math.cos(robotHeading);
        double sinR = Math.sin(robotHeading);

        // Turret pivot in field frame
        double tx = Constants.Turret.kTurretOffsetX;
        double ty = Constants.Turret.kTurretOffsetY;
        double turretFieldX = robotPose.getX() + cosR * tx - sinR * ty;
        double turretFieldY = robotPose.getY() + sinR * tx + cosR * ty;
        double turretHeading = robotHeading + turretAngleRad;
        turretFieldPose.set(new Pose2d(turretFieldX, turretFieldY, new Rotation2d(turretHeading)));

        // Camera in field frame (turret pivot + rotated cam-from-turret offset)
        double cosT = Math.cos(turretHeading);
        double sinT = Math.sin(turretHeading);
        double cx = Constants.Vision.kCamFromTurretX;
        double cy = Constants.Vision.kCamFromTurretY;
        double camFieldX = turretFieldX + cosT * cx - sinT * cy;
        double camFieldY = turretFieldY + sinT * cx + cosT * cy;
        cameraFieldPose.set(new Pose2d(camFieldX, camFieldY, new Rotation2d(turretHeading)));
    }

    /** Clear tag arrays when no targets are visible. Only call when you
     *  know vision returned empty — NOT every frame. */
    public void publishNoTargets() {
        visibleTagPoses.set(EMPTY_POSE3D_ARRAY);
        tagCount.set(0);
    }
}
