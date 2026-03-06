package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class Constants {

    public static class Vision {

        public static final Transform3d kLimelightPosition = new Transform3d(
            new Translation3d(
                Units.inchesToMeters(-11.919),
                Units.inchesToMeters(1.715),
                Units.inchesToMeters(18.0)),  // TODO: Verify that this is from the ground.
            new Rotation3d(
                Units.degreesToRadians(0),      // Roll: 0 degrees
                Units.degreesToRadians(-30.0),  // Pitch: negative = nose up (WPILib NWU convention)
                Units.degreesToRadians(0)       // Yaw: degrees
            ))
        ;

        // Camera XY offset from turret pivot in turret-local frame (meters, 2D only)
        // These use the camera's absolute XY minus the turret pivot's absolute XY
        public static final double kCamFromTurretX =
            Units.inchesToMeters(-11.919) - Turret.kTurretOffsetX;
        public static final double kCamFromTurretY =
            Units.inchesToMeters(1.715) - Turret.kTurretOffsetY;
    }

    public static class Turret {

        public static final int kTurretCanID = 44; // CAN ID (dimensionless)

        // WCP Throughbore CANcoder on output shaft
        // public static final int kThroughboreCanID = 58;
        // public static final double kEncoderOffset = 0.0;  // TODO: calibrate so 0.0 = forward (rotations)

        public static final double kTurretOffsetX  = Units.inchesToMeters(-6.715); // X offset (forward +X, backward -X) from robot center to turret (meters) 
        public static final double kTurretOffsetY = Units.inchesToMeters(6); // Y offset (left +Y, right -Y) from robot center to turret (meters)
        public static final double kTurretOffsetZ = Units.inchesToMeters(14.150); // Z offset (up +Z, down -Z) from robot center to turret (meters)
        public static final double kTurretRezeroAngleDegrees = 0.0; // Rezero position in degrees

        public static final Translation3d kTurretOffsetFromRobotCenter = new Translation3d(kTurretOffsetX, kTurretOffsetY, kTurretOffsetZ);

        // Turret mechanical limits (degrees, relative to robot forward)
        public static final double kMinAngleDegrees = 90;
        public static final double kMaxAngleDegrees = 270;
        public static final Angle kOriginAngle = Degrees.of(180);

        // Shooting range limits (meters)
        public static final double kMinShootingDistance = 1.8288;
        public static final double kMaxShootingDistance = 5.9436;

        // Motor and control constants
        public static final double kGearRatio = (48.0 / 14.0) * (120.0 / 35.0); // (48.0 / 14.0) * (120.0 / 35.0) / 3; // 18t -> 48t (same shaft) -> 35t -> 120t
        public static final double kKP = 25; // Proportional gain (dimensionless)
        public static final double kKI = 0.2; // Integral gain (dimensionless)
        public static final double kKD = 0.8; // Derivative gain (dimensionless)
        public static final double kKS = 3; // Static friction feedforward (volts)
        public static final double kKV = 2; // Velocity feedforward (volt-seconds per radian)
        public static final double kKA = 0; // Acceleration feedforward (volt-seconds² per radian)
        public static final double kKG = 0; // Gravity feedforward (volts) - Unused for turrets
        public static final double kMaxVelocity = 4; // Maximum velocity (rad/s)
        public static final double kMaxAcceleration = 4; // Maximum acceleration (rad/s²)
        public static final boolean kTurretEnabled = true; // Set false if turret is mechanically/electrically broken; holds at min angle
        public static final boolean kBrakeMode = true; // Brake mode enabled (boolean)
        public static final boolean kEnableStatorLimit = true; // Stator current limit enabled (boolean)
        public static final double kStatorCurrentLimit = 40; // Stator current limit (amperes)
        public static final boolean kEnableSupplyLimit = false; // Supply current limit enabled (boolean)
        public static final double kSupplyCurrentLimit = 40; // Supply current limit (amperes)
    }

    public static final class Shooter {
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
    
        public static final double kMaxSpeedRPS       = 80.0;
        public static final double kMinSpeedRPS       = 15.0;
        public static final double kSpeedToleranceRPS = 2.0;
    }

    public static final class Intake {
        public static final int kIntakeMotorCanID = 18;         // KrakenX60 (TalonFX) — intake shaft
        public static final int kExtensionMotorCanID = 53;      // NEO (SparkMax) — linear extension // TODO: confirm CAN ID

        // 14t driving 48t gear
        public static final double kExtensionGearRatio = 48.0 / 14.0; // motor rotations per output rotation

        public static final double kStatorCurrentLimit = 35.0;
        public static final int kExtensionCurrentLimit = 35;       // SparkMax smart current limit (amps)
        public static final double kExtensionStallThreshold = 30;  // stall detection threshold (amps) — must be below kExtensionCurrentLimit
        public static final double kExtensionStallDurationSeconds = 0.25; // sustain overcurrent this long before stopping
        public static final double kHoldSpeed = 0.75; // duty cycle to hold game pieces while extended
    }

    public static final class Drivetrain {
        public static final double kCruiseSpeed = 1.25;
        public static final AngularVelocity kCruiseAngularRate = RotationsPerSecond.of(0.5);
    }

    public static final class Climber {
        // Kraken (TalonFX) — pulls robot up
        public static final double kClimbSpeedPercent = 0.5;
        public static final double kClimbTimeSeconds = 0.75;

        // Spark Max + brushed DC — linear extension to engage climbing structure
        public static final int kExtensionSparkCanID = 20; // TODO: confirm CAN ID
        public static final double kExtensionSpeedPercent = 1.0;
        public static final double kExtensionTimeSeconds = 1.0; // TODO: measure on robot

        // Field pose to drive to before climbing (TODO: measure on field)
        public static final Pose2d kClimbAlignPose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0));

        // PathPlanner constraints for the alignment drive
        public static final PathConstraints kAlignPathConstraints = new PathConstraints(
            1.0, 1.0, Math.PI, Math.PI
        );
    }

}