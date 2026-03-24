package frc.robot.telemetry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.Constants;
import frc.robot.Math.PosePrediction;
import frc.robot.generated.FieldConstants;
import frc.robot.util.dashboard.TurretUtil;

public class ShotTelemetry {
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable shotTable = inst.getTable("Shot");

    private final StructPublisher<Pose2d> shotInterceptPose = shotTable.getStructTopic("InterceptPoint", Pose2d.struct).publish();
    private final StructPublisher<Pose2d> shotPredictedRobotPose = shotTable.getStructTopic("PredictedRobotPose", Pose2d.struct).publish();
    private final DoublePublisher shotDistance = shotTable.getDoubleTopic("DistanceMeters").publish();
    private final DoublePublisher shotTurretAngle = shotTable.getDoubleTopic("TurretAngleDeg").publish();
    private final DoublePublisher shotShooterRPS = shotTable.getDoubleTopic("ShooterRPS").publish();
    private final DoublePublisher shotTrajectoryAngle = shotTable.getDoubleTopic("TrajectoryAngleDeg").publish();
    private final DoublePublisher shotTimeOfFlight = shotTable.getDoubleTopic("TimeOfFlightSec").publish();
    private final BooleanPublisher shotValid = shotTable.getBooleanTopic("Valid").publish();

    /* 3D trajectory arc for shot visualization */
    private static final int TRAJECTORY_SAMPLES = 5;
    private static final double G = 9.81;
    private static final double TRAJECTORY_UPDATE_INTERVAL_S = 0.5;
    private double m_lastTrajectoryUpdateTime = 0;
    private final StructArrayPublisher<Pose3d> trajectoryPub =
            inst.getStructArrayTopic("Field/ShotTrajectory", Pose3d.struct).publish();
    private final StructArrayPublisher<Pose3d> calibratedTrajectoryPub =
            inst.getStructArrayTopic("Field/ShotTrajectoryCalibrated", Pose3d.struct).publish();

    public void publish(Pose2d robotPose, ChassisSpeeds speeds, Rotation2d turretFieldHeading,
                        Translation2d turretTranslation, TurretUtil.ShotSolution shot) {
        if (shot != null) {
            shotDistance.set(shot.distanceMeters);
            shotTurretAngle.set(shot.robotRelativeAngleDegrees);
            shotShooterRPS.set(shot.shooterSpeedRPS);
            shotTrajectoryAngle.set(shot.trajectoryAngleDegrees);
            shotTimeOfFlight.set(shot.timeOfFlightSeconds);
            shotValid.set(shot.isValid);

            // Publish predicted robot pose at TOF
            Pose2d predictedAtShot = PosePrediction.Exponential(robotPose, speeds, shot.timeOfFlightSeconds);
            shotPredictedRobotPose.set(predictedAtShot);

            // Publish intercept point as a Pose2d for visualization
            Rotation2d aimHeading = Rotation2d.fromDegrees(shot.robotRelativeAngleDegrees)
                    .plus(robotPose.getRotation());
            shotInterceptPose.set(new Pose2d(
                    predictedAtShot.getX() + shot.distanceMeters * aimHeading.getCos(),
                    predictedAtShot.getY() + shot.distanceMeters * aimHeading.getSin(),
                    aimHeading));
        }

        /* Publish 3D trajectory arcs when a shot solution exists (throttled) */
        double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        if (now - m_lastTrajectoryUpdateTime >= TRAJECTORY_UPDATE_INTERVAL_S) {
            m_lastTrajectoryUpdateTime = now;
            if (shot != null && shot.isValid) {
                publishTrajectoryArc(turretTranslation, turretFieldHeading, shot);
            } else {
                trajectoryPub.set(new Pose3d[0]);
                calibratedTrajectoryPub.set(new Pose3d[0]);
            }
        }
    }

    private void publishTrajectoryArc(Translation2d turretXY, Rotation2d turretFieldHeading,
                                       TurretUtil.ShotSolution shot) {
        double pitchRad = Math.toRadians(shot.trajectoryAngleDegrees);
        double yawRad = turretFieldHeading.getRadians();
        double launchZ = Constants.Turret.kTurretOffsetZ;
        double cosP = Math.cos(pitchRad);
        double sinP = Math.sin(pitchRad);

        var cal = TurretUtil.getCalibration();
        double targetHeight = FieldConstants.HUB_ENTRANCE_HEIGHT;

        // Physics arc (no drag correction)
        double physicsExitV = cal.getPhysicsRPS(shot.distanceMeters, targetHeight)
                * Math.PI * Constants.Shooter.kWheelDiameterMeters * Constants.Shooter.kSlipFactor;
        trajectoryPub.set(buildArc(turretXY, physicsExitV, cosP, sinP, yawRad, launchZ));

        // Calibrated arc (empirical correction)
        double calibratedExitV = cal.getRPS(shot.distanceMeters, targetHeight)
                * Math.PI * Constants.Shooter.kWheelDiameterMeters * Constants.Shooter.kSlipFactor;
        calibratedTrajectoryPub.set(buildArc(turretXY, calibratedExitV, cosP, sinP, yawRad, launchZ));
    }

    private Pose3d[] buildArc(Translation2d origin, double exitV,
                               double cosP, double sinP, double yawRad, double launchZ) {
        double vx = exitV * cosP * Math.cos(yawRad);
        double vy = exitV * cosP * Math.sin(yawRad);
        double vz = exitV * sinP;

        double tGround = (vz + Math.sqrt(vz * vz + 2 * G * launchZ)) / G;

        Pose3d[] arc = new Pose3d[TRAJECTORY_SAMPLES + 1];
        for (int i = 0; i <= TRAJECTORY_SAMPLES; i++) {
            double t = tGround * i / TRAJECTORY_SAMPLES;
            double x = origin.getX() + vx * t;
            double y = origin.getY() + vy * t;
            double z = launchZ + vz * t - 0.5 * G * t * t;
            if (z < 0) z = 0;

            double curVz = vz - G * t;
            double curPitch = Math.atan2(curVz, exitV * cosP);

            arc[i] = new Pose3d(x, y, z, new Rotation3d(0, -curPitch, yawRad));
        }
        return arc;
    }
}
