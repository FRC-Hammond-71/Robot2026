package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.Timestamp.TimestampSource;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import frc.robot.BufferedStatusSignal;
import frc.robot.Constants;
import frc.robot.Limelight.Limelight;
import frc.robot.Limelight.LimelightHelpers;
import frc.robot.util.LimelightOnTurretUtils;
import frc.robot.utils.ObjectPool;

public class VisionSubsystem {

    // ---- Data records ----

    public static record VisionMeasurement(Pose2d pose, double timestampSeconds) {}

    public static record Telemetry(
        double turretAngleDeg,
        double pigeonHeadingDeg,
        double cameraHeadingDeg,
        Pose2d correctedCameraPose,
        Pose2d finalVisionPose,
        Pose2d odometryPose,
        double correctionDeltaM,
        double visionTimestamp,
        int tagCount,
        double yawRateDegPerSec) {}

    public static record VisionResult(
        VisionMeasurement measurement,
        Telemetry telemetry,
        Optional<Pose2d> rawPose,
        Pose3d[] visibleTagPoses) {}

    // ---- Simulation fields ----

    private final VisionSystemSim visionSim;
    private final PhotonCamera photonCamera;
    private final PhotonCameraSim cameraSim;
    private final AprilTagFieldLayout aprilTagLayout;
    private final Field2d debugField;

    // ---- Pre-allocated pools ----

    private final ObjectPool<Pose3d> tagPosePool = new ObjectPool<>(new Pose3d[0], 8);

    // ---- Constructor ----

    public VisionSubsystem() {
        visionSim = new VisionSystemSim("main");

        AprilTagFieldLayout gameTags = null;
        try {
            gameTags = AprilTagFieldLayout.loadFromResource(
                AprilTagFields.kDefaultField.m_resourceFile);
            visionSim.addAprilTags(gameTags);
        } catch (Exception ex) {
            System.err.println("Warning: could not load AprilTag layout: " + ex.getMessage());
        }
        aprilTagLayout = gameTags;

        debugField = visionSim.getDebugField();

        SimCameraProperties cameraProps = new SimCameraProperties();
        cameraProps.setCalibration(640, 480, Rotation2d.fromDegrees(63));
        cameraProps.setFPS(20);
        cameraProps.setAvgLatencyMs(35);
        cameraProps.setLatencyStdDevMs(5);
        cameraProps.setCalibError(0.25, 0.08);

        photonCamera = new PhotonCamera("limelight");
        cameraSim = new PhotonCameraSim(photonCamera, cameraProps);
        cameraSim.enableRawStream(false);
        cameraSim.enableProcessedStream(false);

        visionSim.addCamera(cameraSim, new Transform3d(
            new Translation3d(
                Constants.Turret.kTurretOffsetX + Constants.Vision.kCamFromTurretX,
                Constants.Turret.kTurretOffsetY + Constants.Vision.kCamFromTurretY,
                Constants.Turret.kTurretOffsetZ + Constants.Vision.kCamFromTurretZ),
            Constants.Vision.kLimelightRotation));
    }

    // ---- Turret angle conversion ----

    /**
     * Converts turret motor rotations to radians.
     * The motor reads 0.5 rotations at 180 deg (backward), 0.25 at 90 deg, etc.
     * Simple rotations-to-radians: rotations * 2pi.
     */
    public static double turretRotationsToRadians(double rotations) {
        return rotations * 2.0 * Math.PI;
    }

    // ---- Real pipeline: Limelight ----

    public Optional<VisionResult> processLimelight(
            Limelight limelight,
            ChassisSpeeds robotVelocity,
            BufferedStatusSignal<edu.wpi.first.units.measure.Angle> bufferedTurretRotations,
            double pigeonYawDeg,
            Pose2d odometryPose,
            double yawRateDegPerSec) {

        if (limelight == null || bufferedTurretRotations == null) {
            return Optional.empty();
        }

        // MegaTag2 requires the robot heading every frame to compute valid poses.
        LimelightHelpers.SetRobotOrientation(limelight.name, pigeonYawDeg, yawRateDegPerSec, 0, 0, 0, 0);

        double robotSpeedMps = Math.hypot(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond);
        Optional<Pose2d> raw = limelight.getRawEstimatedPose();
        Optional<Pose2d> mega = limelight.getStableEstimatedPose(robotSpeedMps);

        if (mega.isEmpty() || mega.get().equals(Pose2d.kZero)) {
            return Optional.empty();
        }

        Pose2d megaPose = mega.get();
        double visionTimestamp = Timer.getFPGATimestamp() - limelight.getLatencyInSeconds();
        double turretRotations = bufferedTurretRotations.getValueAt(
            visionTimestamp, TimestampSource.System);
        int tagCount = limelight.getLastTagCount();

        // Compute the result first to get camera heading for tag vector computation
        double turretAngleRad = turretRotationsToRadians(turretRotations);
        Rotation2d pigeonHeading = Rotation2d.fromDegrees(pigeonYawDeg);
        Rotation2d cameraHeading = pigeonHeading.plus(new Rotation2d(turretAngleRad));
        Pose2d correctedCameraPose = new Pose2d(megaPose.getTranslation(), cameraHeading);

        // Collect visible tag poses from Limelight raw fiducials
        tagPosePool.clear();
        collectLimelightTags(limelight.name);

        return Optional.of(computeVisionResult(
            megaPose, visionTimestamp, turretRotations,
            pigeonYawDeg, odometryPose, yawRateDegPerSec,
            tagCount, raw));
    }

    // ---- Sim pipeline: PhotonVision ----

    public Optional<VisionResult> processSimVision(
            BufferedStatusSignal<edu.wpi.first.units.measure.Angle> bufferedTurretRotations,
            double pigeonYawDeg,
            Pose2d odometryPose,
            double yawRateDegPerSec) {

        if (bufferedTurretRotations == null) {
            return Optional.empty();
        }

        var results = photonCamera.getAllUnreadResults();
        if (results.isEmpty()) {
            return Optional.empty();
        }

        PhotonPipelineResult latestResult = results.get(results.size() - 1);
        if (!latestResult.hasTargets()) {
            return Optional.empty();
        }

        // Try multi-tag first, fall back to best single-tag
        Pose2d cameraPose2d;
        var multiTag = latestResult.getMultiTagResult();
        if (multiTag.isPresent()) {
            // estimatedPose.best is a Transform3d (field-to-camera)
            Transform3d fieldToCamera = multiTag.get().estimatedPose.best;
            cameraPose2d = new Pose3d().transformBy(fieldToCamera).toPose2d();
        } else {
            var bestTarget = latestResult.getBestTarget();
            if (bestTarget == null || aprilTagLayout == null) {
                return Optional.empty();
            }
            var tagPoseOpt = aprilTagLayout.getTagPose(bestTarget.getFiducialId());
            if (tagPoseOpt.isEmpty()) {
                return Optional.empty();
            }
            Pose3d tagPose3d = tagPoseOpt.get();
            Transform3d camToTarget = bestTarget.getBestCameraToTarget();
            Pose3d cameraPose3d = tagPose3d.transformBy(camToTarget.inverse());
            cameraPose2d = cameraPose3d.toPose2d();
        }

        double visionTimestamp = latestResult.getTimestampSeconds();
        double turretRotations = bufferedTurretRotations.getValueAt(
            visionTimestamp, TimestampSource.System);
        int tagCount = latestResult.getTargets().size();

        // Collect visible tag poses
        tagPosePool.clear();
        if (aprilTagLayout != null) {
            for (var target : latestResult.getTargets()) {
                var tagOpt = aprilTagLayout.getTagPose(target.getFiducialId());
                tagOpt.ifPresent(tagPose -> tagPosePool.list().add(tagPose));
            }
        }

        return Optional.of(computeVisionResult(
            cameraPose2d, visionTimestamp, turretRotations,
            pigeonYawDeg, odometryPose, yawRateDegPerSec,
            tagCount, Optional.of(cameraPose2d)));
    }

    // ---- Shared transform logic ----

    private VisionResult computeVisionResult(
            Pose2d rawCameraPose,
            double visionTimestamp,
            double turretRotations,
            double pigeonYawDeg,
            Pose2d odometryPose,
            double yawRateDegPerSec,
            int tagCount,
            Optional<Pose2d> rawPose) {

        double turretAngleRad = turretRotationsToRadians(turretRotations);
        double turretAngleDeg = Math.toDegrees(turretAngleRad);

        Rotation2d pigeonHeading = Rotation2d.fromDegrees(pigeonYawDeg);
        double pigeonHeadingDeg = pigeonHeading.getDegrees();

        Rotation2d cameraHeading = pigeonHeading.plus(new Rotation2d(turretAngleRad));
        double cameraHeadingDeg = cameraHeading.getDegrees();

        Pose2d correctedCameraPose = new Pose2d(
            rawCameraPose.getTranslation(), cameraHeading);

        LimelightOnTurretUtils.TransformDebug debug =
            LimelightOnTurretUtils.getRobotPoseFromCameraPoseDebug(
                correctedCameraPose, turretAngleRad);

        // Final pose uses pigeon heading (more stable than vision heading)
        Pose2d finalPose = new Pose2d(
            debug.robotPose().getTranslation(), pigeonHeading);

        double correctionDelta = finalPose.getTranslation()
            .getDistance(odometryPose.getTranslation());

        Telemetry telemetry = new Telemetry(
            turretAngleDeg, pigeonHeadingDeg, cameraHeadingDeg,
            correctedCameraPose, finalPose, odometryPose,
            correctionDelta, visionTimestamp, tagCount, yawRateDegPerSec);

        VisionMeasurement vm = new VisionMeasurement(finalPose, visionTimestamp);

        return new VisionResult(
            vm, telemetry, rawPose,
            tagPosePool.toArray());
    }

    /** Populates the tag pose pool from Limelight raw fiducials. */
    private void collectLimelightTags(String limelightName) {
        if (aprilTagLayout == null) return;

        try {
            var rawFids = LimelightHelpers.getRawFiducials(limelightName);
            if (rawFids == null) return;

            for (var rf : rawFids) {
                aprilTagLayout.getTagPose((int) rf.id)
                    .ifPresent(p -> tagPosePool.list().add(p));
            }
        } catch (Exception ex) {
            // Ignore NT/parse errors
        }
    }

    // ---- Simulation methods ----

    /**
     * Adjusts the simulated camera transform to account for turret rotation.
     * Rotates the camera around the turret pivot so the simulated camera
     * faces the turret heading. Call before {@link #simUpdate(Pose2d)}.
     */
    public void adjustCameraForTurret(Pose2d robotPose, double turretAngleRad) {
        double turretOffsetX = Constants.Turret.kTurretOffsetX;
        double turretOffsetY = Constants.Turret.kTurretOffsetY;
        double camFromTurretX = Constants.Vision.kCamFromTurretX;
        double camFromTurretY = Constants.Vision.kCamFromTurretY;

        // Rotate camera offset around turret pivot by turret angle
        double cos = Math.cos(turretAngleRad);
        double sin = Math.sin(turretAngleRad);
        double camX = turretOffsetX + cos * camFromTurretX - sin * camFromTurretY;
        double camY = turretOffsetY + sin * camFromTurretX + cos * camFromTurretY;
        double camZ = Constants.Turret.kTurretOffsetZ + Constants.Vision.kCamFromTurretZ;

        Translation3d camTranslation = new Translation3d(camX, camY, camZ);
        Rotation3d camRotation = Constants.Vision.kLimelightRotation
            .rotateBy(new Rotation3d(0, 0, turretAngleRad));

        visionSim.adjustCamera(cameraSim,
            new Transform3d(camTranslation, camRotation));
    }

    /** Run the vision sim update. Call after {@link #adjustCameraForTurret}. */
    public void simUpdate(Pose2d robotPose) {
        visionSim.update(robotPose);
    }

    // ---- Accessors ----

    public Field2d getDebugField() {
        return debugField;
    }

    public PhotonCamera getPhotonCamera() {
        return photonCamera;
    }
}
