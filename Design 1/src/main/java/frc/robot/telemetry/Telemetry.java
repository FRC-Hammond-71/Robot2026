package frc.robot.telemetry;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.dashboard.TurretUtil;

public class Telemetry {
    private final DriveTelemetry driveTelemetry;
    private final TurretTelemetry turretTelemetry;
    private final ShotTelemetry shotTelemetry;
    private final FieldTelemetry fieldTelemetry;

    private Rotation2d m_turretFieldHeading = Rotation2d.kZero;
    private Pose2d m_leadShotPose = new Pose2d();
    private TurretUtil.ShotSolution m_latestShot = null;

    /**
     * Construct a telemetry object, with the specified max speed of the robot
     *
     * @param maxSpeed Maximum speed in meters per second
     */
    public Telemetry(double maxSpeed) {
        SignalLogger.start();
        driveTelemetry = new DriveTelemetry(maxSpeed);
        turretTelemetry = new TurretTelemetry();
        shotTelemetry = new ShotTelemetry();
        fieldTelemetry = new FieldTelemetry();
    }

    /** Set the current turret angle in robot-relative degrees (0 = robot forward, CCW+). */
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

        fieldTelemetry.publish(robotPose);
        driveTelemetry.publish(state);

        Translation2d turretTranslation = turretTelemetry.getTurretTranslation(robotPose);
        turretTelemetry.publish(robotPose, m_turretFieldHeading, m_leadShotPose);
        shotTelemetry.publish(robotPose, state.Speeds, m_turretFieldHeading, turretTranslation, m_latestShot);
    }
}
