package frc.robot.util.dashboard;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.generated.FieldConstants;
import frc.robot.subsystems.Turret;

/**
 * Utility class for turret targeting calculations.
 * All distances and angles are computed from the turret position, not the robot
 * center.
 */
public class TurretUtil {

    // =========================
    // TARGET TYPES
    // =========================

    /** Enum for selecting which lookup table / target to use. */
    public enum TargetType {
        HUB,
        LEFT_PASS,
        RIGHT_PASS
    }

    // =========================
    // DATA CLASS
    // =========================

    /** Immutable snapshot of everything needed to execute a shot. */
    public static class ShotSolution {
        public final double distanceMeters; // Turret-to-target distance (m)
        public final double robotRelativeAngleDegrees; // Required turret angle relative to robot heading (°)
        public final double trajectoryAngleDegrees; // Pivot / hood angle from lookup table (°)
        public final double shooterSpeedRPS; // Flywheel speed from lookup table (RPS)
        public final double timeOfFlightSeconds; // Ball time-of-flight from lookup table (s)
        public final boolean isValid; // True if shot is within range & turret limits

        public ShotSolution(double distanceMeters, double robotRelativeAngleDegrees,
                double trajectoryAngleDegrees, double shooterSpeedRPS,
                double timeOfFlightSeconds, boolean isValid) {
            this.distanceMeters = distanceMeters;
            this.robotRelativeAngleDegrees = robotRelativeAngleDegrees;
            this.trajectoryAngleDegrees = trajectoryAngleDegrees;
            this.shooterSpeedRPS = shooterSpeedRPS;
            this.timeOfFlightSeconds = timeOfFlightSeconds;
            this.isValid = isValid;
        }
    }

    // =========================
    // CONSTANTS & LOOKUP TABLES
    // =========================

    private static final double TURRET_OFFSET_X = Constants.Turret.kTurretOffsetX;
    private static final double TURRET_OFFSET_Y = Constants.Turret.kTurretOffsetY;

    private static final HubLookUpTable hubTable = new HubLookUpTable();
    private static final PassLookUpTable passTable = new PassLookUpTable();

    /** Pre-converted 2D field targets. */
    private static final Pose2d LEFT_PASS_TARGET = FieldConstants.leftPassTarget.toPose2d();
    private static final Pose2d RIGHT_PASS_TARGET = FieldConstants.rightPassTarget.toPose2d();

    // =========================
    // TURRET POSE
    // =========================

    /**
     * Returns the field-relative pose of the turret given the current robot pose.
     * The turret heading is set to the robot heading (turret rotation is handled
     * separately).
     */
    public static Pose2d getTurretPose(Pose2d robotPose) {
        return robotPose.plus(new Transform2d(
                new Translation2d(TURRET_OFFSET_X, TURRET_OFFSET_Y),
                new Rotation2d()));
    }

    // =========================
    // TARGET SELECTION
    // =========================

    /** Returns the 2D field pose of the requested target. */
    public static Pose2d getTargetPose(TargetType target) {
        switch (target) {
            case HUB:
                return new Pose2d(FieldConstants.getAllianceHub().toTranslation2d(), Rotation2d.kZero);
            case LEFT_PASS:
                return LEFT_PASS_TARGET;
            case RIGHT_PASS:
                return RIGHT_PASS_TARGET;
            default:
                return null;
        }
    }

    // =========================
    // DISTANCE & ANGLE (from turret)
    // =========================

    /**
     * Horizontal distance in meters from the turret to the target.
     */
    public static double getDistance(Pose2d robotPose, TargetType target) {
        Translation2d turret = getTurretPose(robotPose).getTranslation();
        Translation2d goal = getTargetPose(target).getTranslation();
        return turret.getDistance(goal);
    }

    /**
     * Field-relative angle (radians) from the turret to the target.
     */
    public static double getFieldAngleToTarget(Pose2d robotPose, TargetType target) {
        Translation2d turret = getTurretPose(robotPose).getTranslation();
        Translation2d goal = getTargetPose(target).getTranslation();
        double dx = goal.getX() - turret.getX();
        double dy = goal.getY() - turret.getY();
        return Math.atan2(dy, dx);
    }

    /**
     * Robot-relative turret angle in degrees to aim at the target.
     * Uses Turret.fieldToRobotRelativeDegrees() — no manual normalization needed.
     * @return Angle in [0°, 360°), directly comparable to turret motor range [90°, 270°]
     */
    public static double getRobotRelativeAngleDegrees(Pose2d robotPose, TargetType target) {
        Rotation2d fieldAngle = new Rotation2d(getFieldAngleToTarget(robotPose, target));
        return Turret.fieldToRobotRelativeDegrees(fieldAngle, robotPose.getRotation());
    }

    // =========================
    // LOOKUP TABLE ACCESSORS
    // =========================

    /** Shooter speed (RPS) from the appropriate lookup table. */
    public static double getShooterSpeed(double distanceMeters, TargetType target) {
        return getTableParams(distanceMeters, target).shooterSpeed;
    }

    /** Trajectory / hood angle (degrees) from the appropriate lookup table. */
    public static double getTrajectoryAngle(double distanceMeters, TargetType target) {
        return getTableParams(distanceMeters, target).trajectoryAngle;
    }

    /** Estimated time-of-flight (seconds) from the appropriate lookup table. */
    public static double getTimeOfFlight(double distanceMeters, TargetType target) {
        return getTableParams(distanceMeters, target).timeOfFlight;
    }

    // =========================
    // COMPLETE SHOT SOLUTION
    // =========================

    /**
     * Computes every parameter needed to take a shot at the given target.
     * All geometry is measured from the turret position.
     *
     * @param robotPose Current robot pose on the field
     * @param target    Which target to shoot at
     * @return A {@link ShotSolution} with all parameters and a validity flag
     */
    public static ShotSolution computeShotSolution(Pose2d robotPose, TargetType target) {
        double dist = getDistance(robotPose, target);
        double robotRelativeAngle = getRobotRelativeAngleDegrees(robotPose, target);

        var params = getTableParams(dist, target);

        SmartDashboard.putNumber("Turret Position", robotRelativeAngle);

        boolean valid = isWithinShootingRange(dist) && isTurretAngleReachable(robotRelativeAngle);

        return new ShotSolution(
                dist,
                robotRelativeAngle,
                params.trajectoryAngle,
                params.shooterSpeed,
                params.timeOfFlight,
                valid);
    }

    // =========================
    // VALIDATION
    // =========================

    /** True if the distance is within the lookup-table range. */
    public static boolean isWithinShootingRange(double distanceMeters) {
        return distanceMeters >= Constants.Turret.kMinShootingDistance && distanceMeters <= Constants.Turret.kMaxShootingDistance;
    }

    /** True if the turret can physically reach the requested angle. */
    public static boolean isTurretAngleReachable(double angleDegrees) {
        SmartDashboard.putNumber("Requested Angle", angleDegrees);
        //please keep the smartdashboard keys relevant 
        return angleDegrees >= Constants.Turret.kMinAngleDegrees
                && angleDegrees <= Constants.Turret.kMaxAngleDegrees;
    }

    // =========================
    // INTERNAL HELPERS
    // =========================

    /** Selects the correct lookup table and returns interpolated parameters. */
    private static HubLookUpTable.ShootingParameters getTableParams(double distanceMeters, TargetType target) {
        switch (target) {
            // case HUB:
            //     return hubTable.getParameters(distanceMeters);
            // case LEFT_PASS:
            // case RIGHT_PASS:
            //     // PassLookUpTable has the same ShootingParameters shape; bridge here
            //     PassLookUpTable.ShootingParameters p = passTable.getParameters(distanceMeters);
            //     return new HubLookUpTable.ShootingParameters(p.shooterSpeed, p.trajectoryAngle, p.timeOfFlight);
            default:
                return hubTable.getParameters(distanceMeters);
        }
    }
}
