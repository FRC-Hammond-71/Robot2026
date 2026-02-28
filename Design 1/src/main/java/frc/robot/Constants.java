package frc.robot;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class Constants {

    public static class Vision {

        public static final Transform3d kLimelightPosition = new Transform3d(
            new Translation3d(
                Units.inchesToMeters(6.715), 
                Units.inchesToMeters(11.919), 
                Units.inchesToMeters(19.195)),  // TODO: Verify that this is from the ground.
            new Rotation3d(
                Units.degreesToRadians(0),      // Roll: 0 degrees
                Units.degreesToRadians(58.19),  // Pitch
                Units.degreesToRadians(0)       // Yaw: degrees
            ))
        ;

        public static final Transform3d kLimelightOffsetFromTurretOffset = new Transform3d(
            kLimelightPosition.getTranslation().minus(Turret.kTurretOffsetFromRobotCenter),
            kLimelightPosition.getRotation()
        );
    }

    public static class Turret {

        public static final int kTurretCanID = 44; // CAN ID (dimensionless)

        public static final double kTurretOffsetX  = Units.inchesToMeters(6.715); // X offset (forward +X, backward -X) from robot center to turret (meters) 
        public static final double kTurretOffsetY = Units.inchesToMeters(5.282); // Y offset (left +Y, right -Y) from robot center to turret (meters)
        public static final double kTurretOffsetZ = Units.inchesToMeters(14.150); // Z offset (up +Z, down -Z) from robot center to turret (meters)
        public static final double kTurretRezeroAngleDegrees = 0.0; // Rezero position in degrees

        public static final Translation3d kTurretOffsetFromRobotCenter = new Translation3d(kTurretOffsetX, kTurretOffsetY, kTurretOffsetZ);

        // Turret mechanical limits (degrees, relative to robot forward)
        public static final double kMinAngleDegrees = -180.0;
        public static final double kMaxAngleDegrees = 180.0;

        // Shooting range limits (meters)
        public static final double kMinShootingDistance = 1.0;
        public static final double kMaxShootingDistance = 5.5;

        // Motor and control constants
        public static final double kGearRatio = 15; // Gear ratio (dimensionless)
        public static final double kKP = 8; // Proportional gain (dimensionless)
        public static final double kKI = 0; // Integral gain (dimensionless)
        public static final double kKD = 0.1; // Derivative gain (dimensionless)
        public static final double kKS = 0; // Static friction feedforward (volts)
        public static final double kKV = 0; // Velocity feedforward (volt-seconds per radian)
        public static final double kKA = 0; // Acceleration feedforward (volt-seconds² per radian)
        public static final double kKG = 0; // Gravity feedforward (volts) - Unused for turrets
        public static final double kMaxVelocity = 4; // Maximum velocity (rad/s)
        public static final double kMaxAcceleration = 4; // Maximum acceleration (rad/s²)
        public static final boolean kBrakeMode = true; // Brake mode enabled (boolean)
        public static final boolean kEnableStatorLimit = true; // Stator current limit enabled (boolean)
        public static final double kStatorCurrentLimit = 40; // Stator current limit (amperes)
        public static final boolean kEnableSupplyLimit = false; // Supply current limit enabled (boolean)
        public static final double kSupplyCurrentLimit = 40; // Supply current limit (amperes)
    }

    public static final class ShooterConstants {
        // Hub field coordinate — confirm from AprilTagFieldLayout 2026 JSON
        public static final Translation2d kHubPosition = new Translation2d(8.774, 4.105);
    
        // Physics model — ALL THREE must be measured on your physical robot
        public static final double kLaunchAngleRad      = Math.toRadians(42.0);
        public static final double kHeightDeltaMeters   = 0.5;    // MEASURE: launcher exit to hub opening (m)
        public static final double kWheelDiameterMeters = 0.1016; // MEASURE: roller contact diameter (m)
        public static final double kSlipFactor          = 0.90;   // TUNE: 0.85–0.95 typical, 1.0 = no slip
        public static final double kG                   = 9.81;
    
        // PID / feedforward — tune on robot
        public static final double kP = 0.15;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kV = 0.12;
    
        public static final double kMaxSpeedRPS       = 100.0;
        public static final double kMinSpeedRPS       = 15.0;
        public static final double kSpeedToleranceRPS = 2.0;
    }

}