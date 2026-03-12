package frc.robot.Math;

import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ShooterMath {

    private static final double GRAVITY = 9.81;

    private final double pitch;
    private final double wheelDiameter;
    private final double efficiency;
    private final double shooterHeight;
    private final double targetHeight;

    public ShooterMath(
            double pitchDeg,
            double wheelDiameter,
            double efficiency,
            double shooterHeight,
            double targetHeight
    ) {
        this.pitch = Math.toRadians(pitchDeg);
        this.wheelDiameter = wheelDiameter;
        this.efficiency = efficiency;
        this.shooterHeight = shooterHeight;
        this.targetHeight = targetHeight;
    }

    /*
     * Distance → required exit velocity
     */
    public double requiredExitVelocity(double distance) {

        double cos = Math.cos(pitch);
        double tan = Math.tan(pitch);

        double dz = shooterHeight - targetHeight;

        double numerator = GRAVITY * distance * distance;
        double denominator = 2 * cos * cos * (distance * tan + dz);

        return Math.sqrt(numerator / denominator);
    }

    /*
     * exit velocity → RPS
     */
    public double velocityToRPS(double velocity) {
        return velocity / (Math.PI * wheelDiameter * efficiency);
    }

    /*
     * RPS → horizontal speed
     */
    public double horizontalSpeed(double rps) {
        double v = rps * Math.PI * wheelDiameter * efficiency;
        return v * Math.cos(pitch);
    }

    /*
     * Closed-form intercept time
     */
    public double interceptTime(
            Translation2d robotPos,
            Translation2d targetPos,
            Translation2d targetVel,
            double projectileSpeed
    ) {

        Translation2d R = targetPos.minus(robotPos);

        double rx = R.getX();
        double ry = R.getY();

        double vx = targetVel.getX();
        double vy = targetVel.getY();

        double a = (vx * vx + vy * vy) - projectileSpeed * projectileSpeed;
        double b = 2 * (rx * vx + ry * vy);
        double c = rx * rx + ry * ry;

        double disc = b * b - 4 * a * c;

        if (disc < 0) return Double.NaN;

        double t1 = (-b + Math.sqrt(disc)) / (2 * a);
        double t2 = (-b - Math.sqrt(disc)) / (2 * a);

        double t = Math.min(t1, t2);

        if (t < 0) t = Math.max(t1, t2);

        return t;
    }

    /*
     * Predict target location
     */
    public Translation2d predictedTarget(
            Translation2d robotPos,
            Translation2d targetPos,
            Translation2d relativeVelocity,
            double projectileSpeed
    ) {

        double t = interceptTime(robotPos, targetPos, relativeVelocity, projectileSpeed);

        if (Double.isNaN(t)) return targetPos;

        return targetPos.plus(relativeVelocity.times(t));
    }

    /*
     * Solve full shot
     */
    public ShotSolution solveShot(
            Translation2d robotPos,
            Translation2d targetPos,
            Translation2d targetVelocity,
            ChassisSpeeds robotVelocity
    ) {

        // Caller must pass field-relative speeds
        Translation2d robotVel = new Translation2d(
                robotVelocity.vxMetersPerSecond,
                robotVelocity.vyMetersPerSecond
        );

        // relative velocity
        Translation2d relativeVel = targetVelocity.minus(robotVel);

        Translation2d futureTarget = targetPos;

        double rps = 0;
        double distance = 0;

        // iterate a few times
        for (int i = 0; i < 3; i++) {

            distance = robotPos.getDistance(futureTarget);

            double exitVelocity = requiredExitVelocity(distance);

            rps = velocityToRPS(exitVelocity);

            double horizontalSpeed = exitVelocity * Math.cos(pitch);

            double t = interceptTime(robotPos, targetPos, relativeVel, horizontalSpeed);

            if (!Double.isNaN(t))
                futureTarget = targetPos.plus(relativeVel.times(t));
        }

        Translation2d diff = futureTarget.minus(robotPos);

        double yaw = Math.atan2(diff.getY(), diff.getX());

        return new ShotSolution(yaw, rps, futureTarget, distance);
    }

    public static class ShotSolution {

        public final double yaw;
        public final double rps;
        public final Translation2d interceptPoint;
        public final double distance;

        public ShotSolution(double yaw, double rps, Translation2d interceptPoint, double distance) {
            this.yaw = yaw;
            this.rps = rps;
            this.interceptPoint = interceptPoint;
            this.distance = distance;
        }
    }
}