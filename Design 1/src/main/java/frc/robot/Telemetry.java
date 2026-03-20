package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.networktables.BooleanPublisher;
import frc.robot.Math.PosePrediction;
import frc.robot.generated.FieldConstants;
import frc.robot.util.dashboard.TurretUtil;

public class Telemetry {
    private final double MaxSpeed;

    /**
     * Construct a telemetry object, with the specified max speed of the robot
     * 
     * @param maxSpeed Maximum speed in meters per second
     */
    public Telemetry(double maxSpeed) {
        MaxSpeed = maxSpeed;
        SignalLogger.start();

        /* Set up the module state Mechanism2d telemetry */
        for (int i = 0; i < 4; ++i) {
            SmartDashboard.putData("Module " + i, m_moduleMechanisms[i]);
        }
    }

    /* What to publish over networktables for telemetry */
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    /* Robot swerve drive state */
    private final NetworkTable driveStateTable = inst.getTable("DriveState");
    private final StructPublisher<Pose2d> drivePose = driveStateTable.getStructTopic("Pose", Pose2d.struct).publish();
    private final StructPublisher<ChassisSpeeds> driveSpeeds = driveStateTable.getStructTopic("Speeds", ChassisSpeeds.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleStates = driveStateTable.getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleTargets = driveStateTable.getStructArrayTopic("ModuleTargets", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModulePosition> driveModulePositions = driveStateTable.getStructArrayTopic("ModulePositions", SwerveModulePosition.struct).publish();
    private final DoublePublisher driveTimestamp = driveStateTable.getDoubleTopic("Timestamp").publish();
    private final DoublePublisher driveOdometryFrequency = driveStateTable.getDoubleTopic("OdometryFrequency").publish();

    private final StructPublisher<Pose2d> predictedRobotPoses500ms = inst.getStructTopic("Robot/Pose/Predictions/500ms", Pose2d.struct).publish();
    private final StructPublisher<Pose2d> predictedRobotPoses1s = inst.getStructTopic("Robot/Pose/Predictions/1s", Pose2d.struct).publish();
    private final StructPublisher<Pose2d> predictedRobotPoses2s = inst.getStructTopic("Robot/Pose/Predictions/2s", Pose2d.struct).publish();

    /* Turret field-space heading (robot translation + turret field heading) */
    private final StructPublisher<Pose2d> turretFieldPose = driveStateTable.getStructTopic("Turret/FieldPose", Pose2d.struct).publish();
    private final StructPublisher<Pose2d> leadShotFieldPose = driveStateTable.getStructTopic("Turret/LeadShotPose", Pose2d.struct).publish();
    private final StructPublisher<Pose3d> launchOriginPose = inst.getStructTopic("Field/LaunchOrigin", Pose3d.struct).publish();
    private Rotation2d m_turretFieldHeading = Rotation2d.kZero;
    private Pose2d m_leadShotPose = new Pose2d();

    /* Leading shot debug telemetry */
    private final NetworkTable shotTable = inst.getTable("Shot");
    private final StructPublisher<Pose2d> shotInterceptPose = shotTable.getStructTopic("InterceptPoint", Pose2d.struct).publish();
    private final StructPublisher<Pose2d> shotPredictedRobotPose = shotTable.getStructTopic("PredictedRobotPose", Pose2d.struct).publish();
    private final DoublePublisher shotDistance = shotTable.getDoubleTopic("DistanceMeters").publish();
    private final DoublePublisher shotTurretAngle = shotTable.getDoubleTopic("TurretAngleDeg").publish();
    private final DoublePublisher shotShooterRPS = shotTable.getDoubleTopic("ShooterRPS").publish();
    private final DoublePublisher shotTrajectoryAngle = shotTable.getDoubleTopic("TrajectoryAngleDeg").publish();
    private final DoublePublisher shotTimeOfFlight = shotTable.getDoubleTopic("TimeOfFlightSec").publish();
    private final BooleanPublisher shotValid = shotTable.getBooleanTopic("Valid").publish();
    private TurretUtil.ShotSolution m_latestShot = null;

    /* Robot pose for field positioning */
    private final NetworkTable table = inst.getTable("Pose");
    private final DoubleArrayPublisher fieldPub = table.getDoubleArrayTopic("robotPose").publish();
    private final DoubleArrayPublisher turretFieldPub = table.getDoubleArrayTopic("turretPose").publish();
    private final StructArrayPublisher<Translation2d> turretToHubPub = driveStateTable.getStructArrayTopic("Turret/TurretToHub", Translation2d.struct).publish();
    private final StringPublisher fieldTypePub = table.getStringTopic(".type").publish();

    /* Mechanisms to represent the swerve module states */
    private final Mechanism2d[] m_moduleMechanisms = new Mechanism2d[] {
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
    };
    /* A direction and length changing ligament for speed representation */
    private final MechanismLigament2d[] m_moduleSpeeds = new MechanismLigament2d[] {
        m_moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
    };
    /* A direction changing and length constant ligament for module direction */
    private final MechanismLigament2d[] m_moduleDirections = new MechanismLigament2d[] {
        m_moduleMechanisms[0].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[1].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[2].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[3].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
    };

    private final double[] m_poseArray = new double[3];

    /** Set the current turret angle in robot-relative degrees (0° = robot forward, CCW+). */
    public void setTurretHeadingField(Rotation2d angleDegrees) {
        m_turretFieldHeading = angleDegrees;
    }

    /** Set the latest leading shot solution for telemetry. */
    public void setShotSolution(TurretUtil.ShotSolution shot) {
        m_latestShot = shot;
    }

    /** Set the field-relative pose representing where the lead shot is aimed. */
    public void setLeadShotPose(Pose2d pose) {
        m_leadShotPose = pose;
    }

    /** Accept the swerve drive state and telemeterize it to SmartDashboard and SignalLogger. */
    public void telemeterize(SwerveDriveState state) {

        Pose2d robotPose = state.Pose;
        Translation2d hub = FieldConstants.getAllianceHub().toTranslation2d();
        SmartDashboard.putNumber("DistanceToHub/Robot", robotPose.getTranslation().getDistance(hub));
        SmartDashboard.putNumber("DistanceToHub/Turret", TurretUtil.getTurretPose(robotPose).getTranslation().getDistance(hub));

        /* Telemeterize the swerve drive state */
        drivePose.set(state.Pose);
        driveSpeeds.set(state.Speeds);
        driveModuleStates.set(state.ModuleStates);
        driveModuleTargets.set(state.ModuleTargets);
        driveModulePositions.set(state.ModulePositions);
        driveTimestamp.set(state.Timestamp);
        driveOdometryFrequency.set(1.0 / state.OdometryPeriod);

        // Pose Predictions
        predictedRobotPoses500ms.set(PosePrediction.Linear(state.Pose, state.Speeds, 0.5));
        predictedRobotPoses1s.set(PosePrediction.Linear(state.Pose, state.Speeds, 1));
        predictedRobotPoses2s.set(PosePrediction.Linear(state.Pose, state.Speeds, 2));

        // predictedRobotPoses1s.set(null);

        // Leading shot debug
        if (m_latestShot != null) {
            shotDistance.set(m_latestShot.distanceMeters);
            shotTurretAngle.set(m_latestShot.robotRelativeAngleDegrees);
            shotShooterRPS.set(m_latestShot.shooterSpeedRPS);
            shotTrajectoryAngle.set(m_latestShot.trajectoryAngleDegrees);
            shotTimeOfFlight.set(m_latestShot.timeOfFlightSeconds);
            shotValid.set(m_latestShot.isValid);

            // Publish predicted robot pose at TOF
            Pose2d predictedAtShot = PosePrediction.Linear(state.Pose, state.Speeds, m_latestShot.timeOfFlightSeconds);
            shotPredictedRobotPose.set(predictedAtShot);

            // Publish intercept point as a Pose2d (heading = turret aim direction) for Field2d visualization
            Rotation2d aimHeading = Rotation2d.fromDegrees(m_latestShot.robotRelativeAngleDegrees)
                    .plus(state.Pose.getRotation());
            shotInterceptPose.set(new Pose2d(
                    predictedAtShot.getX() + m_latestShot.distanceMeters * aimHeading.getCos(),
                    predictedAtShot.getY() + m_latestShot.distanceMeters * aimHeading.getSin(),
                    aimHeading));
        }

        /* Also write to log file */
        SignalLogger.writeStruct("DriveState/Pose", Pose2d.struct, state.Pose);
        SignalLogger.writeStruct("DriveState/Speeds", ChassisSpeeds.struct, state.Speeds);
        SignalLogger.writeStructArray("DriveState/ModuleStates", SwerveModuleState.struct, state.ModuleStates);
        SignalLogger.writeStructArray("DriveState/ModuleTargets", SwerveModuleState.struct, state.ModuleTargets);
        SignalLogger.writeStructArray("DriveState/ModulePositions", SwerveModulePosition.struct, state.ModulePositions);
        SignalLogger.writeDouble("DriveState/OdometryPeriod", state.OdometryPeriod, "seconds");

        /* Telemeterize the pose to a Field2d */
        fieldTypePub.set("Field2d");

        m_poseArray[0] = state.Pose.getX();
        m_poseArray[1] = state.Pose.getY();
        m_poseArray[2] = state.Pose.getRotation().getDegrees();
        fieldPub.set(m_poseArray);

        /* Telemeterize turret field-space heading (turret offset from robot center + turret field heading) */
        Translation2d turretTranslation = TurretUtil.getTurretPose(state.Pose).getTranslation();
        Pose2d turretPose = new Pose2d(turretTranslation, m_turretFieldHeading);
        turretFieldPose.set(turretPose);
        turretFieldPub.set(new double[] { turretPose.getX(), turretPose.getY(), m_turretFieldHeading.getDegrees() });

        // Translation2d hub = FieldConstants.getAllianceHub().toTranslation2d();
        turretToHubPub.set(new Translation2d[] { turretTranslation, hub });

        leadShotFieldPose.set(m_leadShotPose);

        /* Publish 3D launch origin: turret XY from TurretUtil + turret Z height */
        Pose2d turret2d = TurretUtil.getTurretPose(state.Pose);
        launchOriginPose.set(new Pose3d(
                turret2d.getX(),
                turret2d.getY(),
                Constants.Turret.kTurretOffsetZ,
                new Rotation3d(0, 0, m_turretFieldHeading.getRadians())));

        /* Telemeterize each module state to a Mechanism2d */
        for (int i = 0; i < 4; ++i) {
            m_moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
            m_moduleDirections[i].setAngle(state.ModuleStates[i].angle);
            m_moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * MaxSpeed));
        }
    }
}
