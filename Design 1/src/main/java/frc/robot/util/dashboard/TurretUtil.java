package frc.robot.util.dashboard;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Math.ShooterMath;
import frc.robot.Math.ShootingCalibration;
import frc.robot.generated.FieldConstants;
import frc.robot.subsystems.TurretSubsystem;

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
        AllianceHUB,
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

    private static final ShootingCalibration calibration = ShootingCalibration.createDefault();

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
            case AllianceHUB:
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
    public static double getTurretCenterDistance(Pose2d robotPose, TargetType target) {
        Translation2d turret = getTurretPose(robotPose).getTranslation();
        Translation2d goal = getTargetPose(target).getTranslation();
        return turret.getDistance(goal);
    }

    /**
     * Horizontal distance in meters from the robot center to the target.
     */
    public static double getRobotCenterDistance(Pose2d robotPose, TargetType target) {
        return robotPose.getTranslation().getDistance(getTargetPose(target).getTranslation());
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
        return TurretSubsystem.fieldToRobotRelativeDegrees(fieldAngle, robotPose.getRotation());
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
        // NOTE: Use getTurretCenterDistance or getRobotCenterDistance here depending on whether the lookup tables are based on turret-to-target or robot-to-target distance
        double dist = getTurretCenterDistance(robotPose, target);
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

    private static final ShooterMath leadingInterceptSolver = new ShooterMath(
            42,
            Constants.Shooter.kWheelDiameterMeters,
            Constants.Shooter.kSlipFactor,
            Constants.Shooter.kLaunchHeightMeters,
            FieldConstants.HUB_ENTRANCE_HEIGHT);

    /**
     * Computes a leading shot that accounts for robot motion.
     * Uses ShooterMath for projectile intercept and PosePrediction to
     * estimate where the robot will be at time-of-flight.
     */
    public static ShotSolution computeLeadingShot(Pose2d robotPose, ChassisSpeeds robotVelocity, TargetType target) {
        Translation2d targetTranslation = getTargetPose(target).getTranslation();

        // ChassisSpeeds from the drivetrain is robot-relative (vx=forward, vy=left).
        // The intercept solver works in field coordinates, so rotate into field frame.
        double cos = robotPose.getRotation().getCos();
        double sin = robotPose.getRotation().getSin();
        ChassisSpeeds fieldRelativeVelocity = new ChassisSpeeds(
                robotVelocity.vxMetersPerSecond * cos - robotVelocity.vyMetersPerSecond * sin,
                robotVelocity.vxMetersPerSecond * sin + robotVelocity.vyMetersPerSecond * cos,
                robotVelocity.omegaRadiansPerSecond);

        // Solve the intercept using ShooterMath from turret pivot, not robot center
        Translation2d turretPosition = getTurretPose(robotPose).getTranslation();
        ShooterMath.ShotSolution mathSolution = leadingInterceptSolver.solveShot(
                turretPosition,
                targetTranslation,
                new Translation2d(), // stationary target
                fieldRelativeVelocity);

        // solveShot's yaw already accounts for robot velocity — use it directly
        // as the field-relative aim direction from the current pose.
        Rotation2d fieldAngle = new Rotation2d(mathSolution.yaw);
        double robotRelativeAngle = TurretSubsystem.fieldToRobotRelativeDegrees(fieldAngle, robotPose.getRotation());

        // Use calibrated RPS instead of raw physics RPS from ShooterMath
        double targetHeight = getTargetHeight(target);
        double calibratedRPS = calibration.getRPS(mathSolution.distance, targetHeight);
        double calibratedTOF = calibration.getTimeOfFlight(mathSolution.distance, targetHeight);
        double trajectoryAngle = Math.toDegrees(Constants.Shooter.kLaunchAngleRad);

        boolean valid = isWithinShootingRange(mathSolution.distance)
                && isTurretAngleReachable(robotRelativeAngle);

        return new ShotSolution(
                mathSolution.distance,
                robotRelativeAngle,
                trajectoryAngle,
                calibratedRPS,
                calibratedTOF,
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
        return angleDegrees >= Constants.Turret.kMinAngleDegrees
                && angleDegrees <= Constants.Turret.kMaxAngleDegrees;
    }

    // =========================
    // INTERNAL HELPERS
    // =========================

    /** Returns the Z-height (meters above floor) of the given target. */
    private static double getTargetHeight(TargetType target) {
        switch (target) {
            case AllianceHUB:
                return FieldConstants.HUB_ENTRANCE_HEIGHT;
            case LEFT_PASS:
            case RIGHT_PASS:
                return 0.0;
            default:
                return FieldConstants.HUB_ENTRANCE_HEIGHT;
        }
    }

    /** Returns calibrated shooting parameters for the given distance and target. */
    private static ShootingCalibration.ShootingParameters getTableParams(double distanceMeters, TargetType target) {
        return calibration.getParameters(distanceMeters, getTargetHeight(target));
    }

    /** Returns the shared calibration instance for diagnostics. */
    public static ShootingCalibration getCalibration() {
        return calibration;
    }

    /** Returns whichever pass target (LEFT_PASS or RIGHT_PASS) is closest to the robot. */
    public static TargetType getClosestPassTarget(Pose2d robotPose) {
        double distL = robotPose.getTranslation().getDistance(LEFT_PASS_TARGET.getTranslation());
        double distR = robotPose.getTranslation().getDistance(RIGHT_PASS_TARGET.getTranslation());
        return distL <= distR ? TargetType.LEFT_PASS : TargetType.RIGHT_PASS;
    }
}
