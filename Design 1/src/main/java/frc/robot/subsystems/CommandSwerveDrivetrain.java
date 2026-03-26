package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.function.Supplier;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Limelight.Limelight;
 
import frc.robot.BufferedStatusSignal;
 
import com.ctre.phoenix6.Timestamp.TimestampSource;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.measure.Angle;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.odometry.OdometryDiagnostics;
import frc.robot.odometry.TiltCompensatedOdometry;
 
import frc.robot.utils.simulation.MapleSimSwerveDrivetrain;

// #region WARNING

// Without explicit permission, Do NOT edit this code. Any changed made to this code will be reverted. - Lune
// We don't even know how this code works. do NOT. EDIT. IT.

public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.004; // 4 ms
    private Notifier m_simNotifier = null;
    RobotConfig config;
    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /** Swerve request to apply during robot-centric path following */
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();
    private BufferedStatusSignal<Angle> m_bufferedTurretAngle = null;
    private final TiltCompensatedOdometry tiltCompensatedOdometry = new TiltCompensatedOdometry();
    private final OdometryDiagnostics odometryDiagnostics = new OdometryDiagnostics();

    public BufferedStatusSignal<Angle> getBufferedTurretAngle() {
        return m_bufferedTurretAngle;
    }

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    /*
     * SysId routine for characterizing translation. This is used to find PID gains
     * for the drive motors.
     */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> setControl(m_translationCharacterization.withVolts(output)),
                    null,
                    this));

    /*
     * SysId routine for characterizing steer. This is used to find PID gains for
     * the steer motors.
     */
    @SuppressWarnings("unused")
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(7), // Use dynamic voltage of 7 V
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    volts -> setControl(m_steerCharacterization.withVolts(volts)),
                    null,
                    this));

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle
     * HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on
     * importing the log to SysId.
     */
    @SuppressWarnings("unused")
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    /* This is in radians per second², but SysId only supports "volts per second" */
                    Volts.of(Math.PI / 6).per(Second),
                    /* This is in radians per second, but SysId only supports "volts" */
                    Volts.of(Math.PI),
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> {
                        /* output is actually radians per second, but SysId only supports "volts" */
                        setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                        /* also log the requested output for SysId */
                        SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                    },
                    null,
                    this));

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public void setBufferedTurretAngle(StatusSignal<Angle> turretPositionSignal) {
        m_bufferedTurretAngle = new BufferedStatusSignal<>(turretPositionSignal, TimestampSource.System, 20, 2000);
    }

    public void updateOdometry() {
    }

    public void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();

            AutoBuilder.configure(
                    () -> getState().Pose, // Supplier of current robot pose
                    // this::resetPose, // Seed odometry pose at auto start
                    (resetTo) -> {},
                    () -> getState().Speeds, // Supplier of current robot speeds
                    // Consumer of ChassisSpeeds and feedforwards to drive the robot
                    (speeds, feedforwards) -> setControl(
                            m_pathApplyRobotSpeeds.withSpeeds(ChassisSpeeds.discretize(speeds, 0.020))
                                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                                    ),
                    new PPHolonomicDriveController(
                            new PIDConstants(3, 0, 0),  // translation
                            new PIDConstants(Math.PI / 2, 0, 0)), // rotation
                    config,
                    // Assume the path needs to be flipped for Red vs Blue, this is normally the
                    // case
                    () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                    this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder",
                    ex.getStackTrace());
        }
    };

    /**
     * Returns the robot's current velocity in field-relative coordinates (x
     * forward, y left).
     */
    public ChassisSpeeds getFieldRelativeSpeeds() {
        ChassisSpeeds robotSpeeds = getState().Speeds;
        Rotation2d heading = Rotation2d.fromDegrees(getPigeon2().getYaw().getValueAsDouble());
        double cos = heading.getCos();
        double sin = heading.getSin();
        double vxField = robotSpeeds.vxMetersPerSecond * cos - robotSpeeds.vyMetersPerSecond * sin;
        double vyField = robotSpeeds.vxMetersPerSecond * sin + robotSpeeds.vyMetersPerSecond * cos;
        return new ChassisSpeeds(vxField, vyField, robotSpeeds.omegaRadiansPerSecond);
    }

    public boolean resetPoseWithLimelight() {
        Optional<Limelight> limelightOpt = Limelight.useDevice("limelight");
        if (limelightOpt.isEmpty()) return false;

        Optional<Pose2d> es = limelightOpt.get().getRawEstimatedPose();
        if (es.isPresent()) {
            this.resetPose(es.get());
            return true;
        }
        return false;
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules) {
        super(
                drivetrainConstants,
                odometryUpdateFrequency,
                // modules
                MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(modules));
        if (Utils.isSimulation()) {
            // start simulation thread; VisionSubsystem will be injected by Robot
            startSimThread();
        }
        configureAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve
     *                                  drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz
     *                                  on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry
     *                                  calculation
     *                                  in the form [x, y, theta]ᵀ, with units in
     *                                  meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision
     *                                  calculation
     *                                  in the form [x, y, theta]ᵀ, with units in
     *                                  meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules) {
        super(
                drivetrainConstants,
                odometryUpdateFrequency,
                odometryStandardDeviation,
                visionStandardDeviation,
                // modules
                MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(modules));
        if (Utils.isSimulation()) {
            // start simulation thread; VisionSubsystem will be injected by Robot
            startSimThread();
        }
        configureAutoBuilder();
    }

    /**
     * Returns a command that applies the specified control request to this swerve
     * drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> request) {
        return run(() -> this.setControl(request.get()));
    }

    @Override
    public void resetPose(Pose2d pose) {
        if (this.mapleSimSwerveDrivetrain != null)
            mapleSimSwerveDrivetrain.mapleSimDrive.setSimulationWorldPose(pose);
        Timer.delay(0.05); // Wait for simulation to update
        super.resetPose(pose);
        tiltCompensatedOdometry.reset(pose);
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply
         * it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts
         * mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is
         * disabled.
         * This ensures driving behavior doesn't change until an explicit disable event
         * occurs during testing.
         */

        SmartDashboard.putBoolean("Pigeon Connected", getPigeon2().isConnected());

        SmartDashboard.putNumber("CommandSwerveDrivetrain/Pigeon/Heading",
                getPigeon2().getYaw(true).getValue().in(Degrees));
        SmartDashboard.putNumber("CommandSwerveDrivetrain/Fused/Heading", getState().Pose.getRotation().getDegrees());

        if (m_bufferedTurretAngle != null)
            m_bufferedTurretAngle.periodic();

        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? kRedAlliancePerspectiveRotation
                                : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
            });
        }

        updateOdometry();

        // Tilt compensation
        double pitchDeg = getPigeon2().getPitch().getValueAsDouble();
        double rollDeg = getPigeon2().getRoll().getValueAsDouble();
        tiltCompensatedOdometry.update(getState().Pose, pitchDeg, rollDeg);
        SmartDashboard.putNumber("Odometry/TiltFactor",
            TiltCompensatedOdometry.tiltFactor(pitchDeg, rollDeg));

        // Odometry diagnostics
        odometryDiagnostics.publish(
            tiltCompensatedOdometry.getCompensatedPose(),
            getState().Pose, pitchDeg, rollDeg);

        // Per-module telemetry
        var state = getState();
        for (int i = 0; i < 4; i++) {
            String prefix = "Module/" + i + "/";
            double steerAngleDeg = state.ModuleStates[i].angle.getDegrees();
            double targetAngleDeg = state.ModuleTargets[i].angle.getDegrees();
            double steerError = steerAngleDeg - targetAngleDeg;
            steerError = ((steerError + 180) % 360 + 360) % 360 - 180;

            SmartDashboard.putNumber(prefix + "SteerAngleDeg", steerAngleDeg);
            SmartDashboard.putNumber(prefix + "TargetAngleDeg", targetAngleDeg);
            SmartDashboard.putNumber(prefix + "SteerErrorDeg", steerError);
            SmartDashboard.putNumber(prefix + "DrivePositionM", state.ModulePositions[i].distanceMeters);
            SmartDashboard.putNumber(prefix + "DriveVelocityMPS", state.ModuleStates[i].speedMetersPerSecond);
        }
        SmartDashboard.putNumber("Pigeon/PitchDeg", getPigeon2().getPitch().getValueAsDouble());
        SmartDashboard.putNumber("Pigeon/RollDeg", getPigeon2().getRoll().getValueAsDouble());

    }

    // https://github.com/Shenzhen-Robotics-Alliance/CTRE-Swerve-MapleSim/tree/main?tab=readme-ov-file#modify-the-drive-subsystem-code
    private MapleSimSwerveDrivetrain mapleSimSwerveDrivetrain = null;
    private void startSimThread() {
        mapleSimSwerveDrivetrain = new MapleSimSwerveDrivetrain(
                Seconds.of(kSimLoopPeriod),
                // TODO: modify the following constants according to your robot
                Pounds.of(115), // robot weight
                Inches.of(30), // bumper length
                Inches.of(30), // bumper width
                DCMotor.getKrakenX60(1), // drive motor type
                DCMotor.getFalcon500(1), // steer motor type
                1.2, // wheel COF
                getModuleLocations(),
                getPigeon2(),
                getModules(),
                TunerConstants.FrontLeft,
                TunerConstants.FrontRight,
                TunerConstants.BackLeft,
                TunerConstants.BackRight);
        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            // Update drivetrain sim
            mapleSimSwerveDrivetrain.update();

        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public SwerveDriveSimulation getMapleSimSwerveDrivetrain() {
        return mapleSimSwerveDrivetrain.mapleSimDrive;
    }

    /** Returns the tilt-compensated wheel-only pose (no vision fusion). */
    public Pose2d getTiltCompensatedPose() {
        return tiltCompensatedOdometry.getCompensatedPose();
    }

    /*
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision
     * camera.
     * 
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters    The pose of the robot as measured by the
     *                                 vision camera.
     * @param timestampSeconds         The timestamp of the vision measurement in
     *                                 seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose
     *                                 measurement
     *                                 in the form [x, y, theta]ᵀ, with units in
     *                                 meters and radians.
     */
    @Override
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds),
                visionMeasurementStdDevs);
    }

    /**
     * Return the pose at a given timestamp, if the buffer is not empty.
     *
     * @param timestampSeconds The timestamp of the pose in seconds.
     * @return The pose at the given timestamp (or Optional.empty() if the buffer is
     *         empty).
     */
    @Override
    public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
        return super.samplePoseAt(Utils.fpgaToCurrentTime(timestampSeconds));
    }

    // Ultimately a failed implementation of auto :(
    // Lune needs to INSPECT IMMEDIATELY before ACTIVATING SWERVE
}