package frc.robot.util.dashboard;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.generated.FieldConstants;

/**
 * Utility class for turret targeting calculations.
 * All distances and angles are computed from the turret position, not the robot center.
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
        public final double distanceMeters;       // Turret-to-target distance (m)
        public final double turretAngleDegrees;   // Required turret angle relative to robot heading (°)
        public final double trajectoryAngleDegrees; // Pivot / hood angle from lookup table (°)
        public final double shooterSpeedRPS;      // Flywheel speed from lookup table (RPS)
        public final double timeOfFlightSeconds;  // Ball time-of-flight from lookup table (s)
        public final boolean isValid;             // True if shot is within range & turret limits

        public ShotSolution(double distanceMeters, double turretAngleDegrees,
                            double trajectoryAngleDegrees, double shooterSpeedRPS,
                            double timeOfFlightSeconds, boolean isValid) {
            this.distanceMeters = distanceMeters;
            this.turretAngleDegrees = turretAngleDegrees;
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
    private static final Pose2d HUB_TARGET      = FieldConstants.hubTarget.toPose2d();
    private static final Pose2d LEFT_PASS_TARGET = FieldConstants.leftPassTarget.toPose2d();
    private static final Pose2d RIGHT_PASS_TARGET = FieldConstants.rightPassTarget.toPose2d();

    // =========================
    // TURRET POSE
    // =========================

    /**
     * Returns the field-relative pose of the turret given the current robot pose.
     * The turret heading is set to the robot heading (turret rotation is handled separately).
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
            case HUB:        return HUB_TARGET;
            case LEFT_PASS:  return LEFT_PASS_TARGET;
            case RIGHT_PASS: return RIGHT_PASS_TARGET;
            default:         return HUB_TARGET;
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
     * Turret angle in degrees that the turret must rotate to, relative to the robot's heading.
     * 0° = robot forward, positive = counter-clockwise.
     */
    public static double getTurretAngleDegrees(Pose2d robotPose, TargetType target) {
        double fieldAngleRad = getFieldAngleToTarget(robotPose, target);
        double robotHeadingRad  = robotPose.getRotation().getRadians();
        double turretRad = fieldAngleRad - robotHeadingRad;
        return normalizeDegrees(Math.toDegrees(turretRad));
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
        double turretAngle = getTurretAngleDegrees(robotPose, target);

        var params = getTableParams(dist, target);

        boolean valid = isWithinShootingRange(dist) && isTurretAngleReachable(turretAngle);

        return new ShotSolution(
                dist,
                turretAngle,
                params.trajectoryAngle,
                params.shooterSpeed,
                params.timeOfFlight,
                valid);
    }

    // =========================
    // MOVING-TARGET LEAD
    // =========================

    /**
     * Computes a lead-compensated shot solution for a robot that is moving while shooting.
     * Predicts where the ball will arrive using the time-of-flight from a static solution,
     * then adjusts the turret angle to aim at the lead-corrected target point.
     *
     * @param robotPose    Current robot field pose
     * @param robotVelX    Robot X velocity on the field (m/s)
     * @param robotVelY    Robot Y velocity on the field (m/s)
     * @param target       Which target to shoot at
     * @return A lead-adjusted {@link ShotSolution}
     */
    public static ShotSolution computeLeadShotSolution(Pose2d robotPose,
                                                        double robotVelX,
                                                        double robotVelY,
                                                        TargetType target) {
        // 1) Get a first-pass static solution for time-of-flight estimate
        double staticDist = getDistance(robotPose, target);
        double tof = getTimeOfFlight(staticDist, target);

        // 2) Predict where the turret will be after time-of-flight
        Pose2d turretNow = getTurretPose(robotPose);
        double futureTurretX = turretNow.getX() + robotVelX * tof;
        double futureTurretY = turretNow.getY() + robotVelY * tof;

        // 3) Recalculate distance from predicted turret position to target
        Translation2d futureTranslation = new Translation2d(futureTurretX, futureTurretY);
        Translation2d goalTranslation = getTargetPose(target).getTranslation();
        double leadDist = futureTranslation.getDistance(goalTranslation);

        // 4) Turret angle to hit target from current pose, aiming where the ball needs to go
        double dx = goalTranslation.getX() - futureTurretX;
        double dy = goalTranslation.getY() - futureTurretY;
        double leadFieldAngle = Math.atan2(dy, dx);
        double turretAngle = normalizeDegrees(
                Math.toDegrees(leadFieldAngle - robotPose.getRotation().getRadians()));

        var params = getTableParams(leadDist, target);
        boolean valid = isWithinShootingRange(leadDist) && isTurretAngleReachable(turretAngle);

        return new ShotSolution(
                leadDist,
                turretAngle,
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
        return distanceMeters >= Constants.Turret.kMinShootingDistance
                && distanceMeters <= Constants.Turret.kMaxShootingDistance;
    }

    /** True if the turret can physically reach the requested angle. */
    public static boolean isTurretAngleReachable(double angleDegrees) {
        return angleDegrees >= Constants.Turret.kMinAngleDegrees
                && angleDegrees <= Constants.Turret.kMaxAngleDegrees;
    }

    // =========================
    // INTERNAL HELPERS
    // =========================

    /** Selects the correct lookup table and returns interpolated parameters. */
    private static HubLookUpTable.ShootingParameters getTableParams(double distanceMeters, TargetType target) {
        switch (target) {
            case HUB:
                return hubTable.getParameters(distanceMeters);
            case LEFT_PASS:
            case RIGHT_PASS:
                // PassLookUpTable has the same ShootingParameters shape; bridge here
                PassLookUpTable.ShootingParameters p = passTable.getParameters(distanceMeters);
                return new HubLookUpTable.ShootingParameters(p.shooterSpeed, p.trajectoryAngle, p.timeOfFlight);
            default:
                return hubTable.getParameters(distanceMeters);
        }
    }

    /** Normalizes an angle to the range [-180, 180) degrees. */
    private static double normalizeDegrees(double degrees) {
        degrees %= 360.0;
        if (degrees >= 180.0)  degrees -= 360.0;
        if (degrees < -180.0)  degrees += 360.0;
        return degrees;
    }
}