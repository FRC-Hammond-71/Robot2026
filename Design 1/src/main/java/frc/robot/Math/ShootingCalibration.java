package frc.robot.Math;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.generated.FieldConstants;

/**
 * Unified shooting calibration: physics-based model with empirical correction.
 *
 * The pure projectile-motion model ignores air drag, so it underpredicts
 * required RPS. Empirical data points (hub hits and/or ground landings)
 * are used to derive a linear correction factor:
 *     correction(d) = a + b * d
 * Then:
 *     calibratedRPS(d, h) = physicsRPS(d, h) * correction(d)
 *
 * This works for any target height because drag is primarily a function
 * of horizontal distance, not vertical delta.
 */
public class ShootingCalibration {

    private static final double G = 9.81;

    private final double pitchRad;
    private final double wheelDiameterM;
    private final double slipFactor;
    private final double launchHeightM;

    private final List<CalibrationPoint> dataPoints = new ArrayList<>();

    // Derived correction: correction(d) = corrA + corrB * d
    private double corrA = 1.0;
    private double corrB = 0.0;

    /** A single empirical calibration measurement. */
    private static class CalibrationPoint {
        final double distance;
        final double empiricalRPS;
        final double targetHeight;

        CalibrationPoint(double distance, double empiricalRPS, double targetHeight) {
            this.distance = distance;
            this.empiricalRPS = empiricalRPS;
            this.targetHeight = targetHeight;
        }
    }

    /** Result structure — drop-in replacement for the old lookup table parameters. */
    public static class ShootingParameters {
        public final double shooterSpeed;     // RPS
        public final double trajectoryAngle;  // Degrees (fixed at launch angle)
        public final double timeOfFlight;     // Seconds

        public ShootingParameters(double shooterSpeed, double trajectoryAngle, double timeOfFlight) {
            this.shooterSpeed = shooterSpeed;
            this.trajectoryAngle = trajectoryAngle;
            this.timeOfFlight = timeOfFlight;
        }
    }

    public ShootingCalibration(double pitchDeg, double wheelDiameterM,
                               double slipFactor, double launchHeightM) {
        this.pitchRad = Math.toRadians(pitchDeg);
        this.wheelDiameterM = wheelDiameterM;
        this.slipFactor = slipFactor;
        this.launchHeightM = launchHeightM;
    }

    // ========================
    // DATA ENTRY
    // ========================

    /** Record a hub shot: ball scored at the given distance and RPS. */
    public void addHubShot(double distanceM, double empiricalRPS) {
        dataPoints.add(new CalibrationPoint(distanceM, empiricalRPS, FieldConstants.HUB_ENTRANCE_HEIGHT));
    }

    /** Record a ground shot: ball launched at the given RPS landed at the measured distance. */
    public void addGroundShot(double empiricalRPS, double landingDistanceM) {
        dataPoints.add(new CalibrationPoint(landingDistanceM, empiricalRPS, 0.0));
    }

    /**
     * Recompute the linear correction coefficients from all calibration points.
     * Must be called after adding data points.
     */
    public void buildCorrectionCurve() {
        int n = dataPoints.size();
        if (n == 0) {
            corrA = 1.0;
            corrB = 0.0;
            return;
        }
        if (n == 1) {
            CalibrationPoint p = dataPoints.get(0);
            double physics = physicsRPS(p.distance, p.targetHeight);
            corrA = (physics > 0) ? p.empiricalRPS / physics : 1.0;
            corrB = 0.0;
            return;
        }

        double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
        for (CalibrationPoint p : dataPoints) {
            double physics = physicsRPS(p.distance, p.targetHeight);
            if (physics <= 0) continue;
            double ratio = p.empiricalRPS / physics;
            sumX += p.distance;
            sumY += ratio;
            sumXY += p.distance * ratio;
            sumX2 += p.distance * p.distance;
        }

        double denom = n * sumX2 - sumX * sumX;
        if (Math.abs(denom) < 1e-9) {
            corrA = sumY / n;
            corrB = 0.0;
        } else {
            corrB = (n * sumXY - sumX * sumY) / denom;
            corrA = (sumY - corrB * sumX) / n;
        }
    }

    // ========================
    // QUERY METHODS
    // ========================

    /** Calibrated RPS for a target at the given distance and height. */
    public double getRPS(double distanceM, double targetHeightM) {
        double physics = physicsRPS(distanceM, targetHeightM);
        return physics * correctionFactor(distanceM);
    }

    /** Physics-estimated time of flight using the corrected exit velocity. */
    public double getTimeOfFlight(double distanceM, double targetHeightM) {
        double correctedRPS = getRPS(distanceM, targetHeightM);
        double correctedExitV = correctedRPS * Math.PI * wheelDiameterM * slipFactor;
        double horizontalV = correctedExitV * Math.cos(pitchRad);
        if (horizontalV <= 0) return 1.0;
        return distanceM / horizontalV;
    }

    /** Full parameters for compatibility with TurretUtil. */
    public ShootingParameters getParameters(double distanceM, double targetHeightM) {
        return new ShootingParameters(
                getRPS(distanceM, targetHeightM),
                Math.toDegrees(pitchRad),
                getTimeOfFlight(distanceM, targetHeightM));
    }

    // ========================
    // DIAGNOSTICS
    // ========================

    /** Returns the correction factor at a given distance. */
    public double getCorrectionAt(double distanceM) {
        return correctionFactor(distanceM);
    }

    /** Exposes raw physics RPS prediction (before correction). */
    public double getPhysicsRPS(double distanceM, double targetHeightM) {
        return physicsRPS(distanceM, targetHeightM);
    }

    /** Returns (corrected - physics) / physics * 100 at the given distance/height. */
    public double getPercentDifference(double distanceM, double targetHeightM) {
        double physics = physicsRPS(distanceM, targetHeightM);
        if (physics <= 0) return 0;
        double corrected = getRPS(distanceM, targetHeightM);
        return (corrected - physics) / physics * 100.0;
    }

    /** Publishes correction curve and per-point diagnostics to SmartDashboard. */
    public void publishDiagnostics() {
        SmartDashboard.putNumber("Calibration/CorrectionA", corrA);
        SmartDashboard.putNumber("Calibration/CorrectionB", corrB);

        for (int i = 0; i < dataPoints.size(); i++) {
            CalibrationPoint p = dataPoints.get(i);
            double physics = physicsRPS(p.distance, p.targetHeight);
            double pctDiff = (physics > 0) ? (p.empiricalRPS - physics) / physics * 100.0 : 0;

            String prefix = "Calibration/Point_" + i + "/";
            SmartDashboard.putNumber(prefix + "Distance", p.distance);
            SmartDashboard.putNumber(prefix + "EmpiricalRPS", p.empiricalRPS);
            SmartDashboard.putNumber(prefix + "PhysicsRPS", physics);
            SmartDashboard.putNumber(prefix + "PctDifference", pctDiff);
        }

        SmartDashboard.putNumber("Calibration/PointCount", dataPoints.size());
    }

    // ========================
    // INTERNAL PHYSICS
    // ========================

    private double physicsExitVelocity(double distanceM, double targetHeightM) {
        double cosP = Math.cos(pitchRad);
        double tanP = Math.tan(pitchRad);
        double dz = launchHeightM - targetHeightM;

        double numerator = G * distanceM * distanceM;
        double denominator = 2.0 * cosP * cosP * (distanceM * tanP + dz);

        if (denominator <= 0) return Double.NaN;
        return Math.sqrt(numerator / denominator);
    }

    private double physicsRPS(double distanceM, double targetHeightM) {
        double exitV = physicsExitVelocity(distanceM, targetHeightM);
        if (Double.isNaN(exitV)) return Constants.Shooter.kMinSpeedRPS;
        return exitV / (Math.PI * wheelDiameterM * slipFactor);
    }

    private double correctionFactor(double distanceM) {
        double c = corrA + corrB * distanceM;
        return Math.max(c, 1.0);
    }

    // ========================
    // FACTORY
    // ========================

    public static ShootingCalibration createHubDefault() {
        ShootingCalibration cal = new ShootingCalibration(
                42.0,
                Constants.Shooter.kWheelDiameterMeters,
                Constants.Shooter.kSlipFactor,
                Constants.Shooter.kLaunchHeightMeters);

        // Empirical hub data: (distance in meters, RPS that scored)
        cal.addHubShot(2.75, 37.0);
        cal.addHubShot(3.00, 37.8);
        cal.addHubShot(3.20, 39.0);
        cal.addHubShot(3.50, 40.0);
        cal.addHubShot(3.80, 41.6);
        cal.addHubShot(4.00, 42.0);
        cal.addHubShot(4.25, 42.0);
        cal.addHubShot(4.50, 45.0);
        cal.addHubShot(4.75, 47.0);
        cal.addHubShot(5.00, 49.0);

        cal.buildCorrectionCurve();

        return cal;
    }

    public static ShootingCalibration createPassingDefault() {
        ShootingCalibration cal = new ShootingCalibration(
                42.0,
                Constants.Shooter.kWheelDiameterMeters,
                Constants.Shooter.kSlipFactor,
                Constants.Shooter.kLaunchHeightMeters);

        cal.addGroundShot(42.5, 5.0); // TEMP: Extrapolated
        cal.addGroundShot(46.9, 6.0); // TEMP: Extrapolated
        cal.addGroundShot(48.0, 6.254); // Measured
        cal.addGroundShot(51.0, 7.0); // TEMP: Extrapolated
        cal.addGroundShot(54.8, 8.0); // TEMP: Extrapolated

        cal.buildCorrectionCurve();

        return cal;
    }
}
