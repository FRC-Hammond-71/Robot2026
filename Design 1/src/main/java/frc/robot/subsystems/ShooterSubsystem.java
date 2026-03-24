package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.SubsystemWithMapleSimSimulation;
import frc.robot.generated.FieldConstants;
import frc.robot.util.dashboard.TurretUtil;

public class ShooterSubsystem extends SubsystemWithMapleSimSimulation {

    private final TalonFX m_motorA = new TalonFX(41);
    private final TalonFX m_motorB = new TalonFX(42);
    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0).withSlot(0);

    private final StatusSignal<AngularVelocity> m_velA = m_motorA.getVelocity();
    private final StatusSignal<AngularVelocity> m_velB = m_motorB.getVelocity();
    private final StatusSignal<Current> m_curA = m_motorA.getStatorCurrent();
    private final StatusSignal<Current> m_curB = m_motorB.getStatorCurrent();

    public ShooterSubsystem(frc.robot.Robot robotInstance) {

        super(robotInstance);

        TalonFXConfiguration cfgA = new TalonFXConfiguration();
        cfgA.CurrentLimits.StatorCurrentLimit = 40;
        cfgA.CurrentLimits.StatorCurrentLimitEnable = true;
        Slot0Configs pid = cfgA.Slot0;
        pid.kP = Constants.Shooter.kP;
        pid.kI = Constants.Shooter.kI;
        pid.kD = Constants.Shooter.kD;
        pid.kV = Constants.Shooter.kV;
        m_motorA.getConfigurator().apply(cfgA);

        // Motor B: same PID, NO hardware inversion — handled in software
        TalonFXConfiguration cfgB = new TalonFXConfiguration();
        cfgB.CurrentLimits.StatorCurrentLimit = 40;
        cfgB.CurrentLimits.StatorCurrentLimitEnable = true;
        Slot0Configs pidB = cfgB.Slot0;
        pidB.kP = Constants.Shooter.kP;
        pidB.kI = Constants.Shooter.kI;
        pidB.kD = Constants.Shooter.kD;
        pidB.kV = Constants.Shooter.kV;
        m_motorB.getConfigurator().apply(cfgB);
    }

    private double desiredRPS;

    public double getDesiedRPS() {
        return this.desiredRPS;
    }

    // --- Velocity control ---
    public void setVelocity(double rps) {

        double clamped = rps == 0 ? 0
                : Math.max(Constants.Shooter.kMinSpeedRPS, Math.min(Constants.Shooter.kMaxSpeedRPS, rps));

        desiredRPS = clamped;

        m_motorA.setControl(m_velocityRequest.withVelocity(-clamped));
        m_motorB.setControl(m_velocityRequest.withVelocity(clamped)); // software inversion
    }

    public void stop() {
        desiredRPS = 0;
        m_motorA.set(0);
        m_motorB.set(0);
    }

    // --- Ready check ---
    public boolean isAtSpeed(double targetRPS) {
        return Math.abs(m_velA.getValueAsDouble() + targetRPS) <= Constants.Shooter.kSpeedToleranceRPS
                && Math.abs(m_velB.getValueAsDouble() - targetRPS) <= Constants.Shooter.kSpeedToleranceRPS;
    }

    /**
     * Calculates required flywheel RPS from horizontal distance.
     * Delegates to the unified ShootingCalibration system which applies
     * empirical correction to the physics model.
     */
    public double calculateTargetRPS(double distanceMeters) {
        double rps = TurretUtil.getShooterSpeed(distanceMeters, TurretUtil.TargetType.AllianceHUB);

        SmartDashboard.putBoolean("Shooter/PhysicsValid", true);
        SmartDashboard.putNumber("Shooter/CalibratedRPS", rps);

        return rps;
    }

    // --- Distance from robot pose to hub center ---
    public double calculateDistance(Translation2d robotTranslation) {
        return robotTranslation.getDistance(FieldConstants.getAllianceHub().toTranslation2d());
    }

    @Override
    protected void periodicReal() {

        BaseStatusSignal.refreshAll(m_velA, m_velB, m_curA, m_curB);

        SmartDashboard.putNumber("Shooter/VelocityA_RPS", m_velA.getValueAsDouble());
        SmartDashboard.putNumber("Shooter/VelocityB_RPS", m_velB.getValueAsDouble());
        SmartDashboard.putNumber("Shooter/CurrentA", m_curA.getValueAsDouble());
        SmartDashboard.putNumber("Shooter/CurrentB", m_curB.getValueAsDouble());

    }

    private double lastShotTime = 0;

    @Override
    protected void periodicSimulated() {

        if (desiredRPS > 0) {

            double now = Timer.getFPGATimestamp();

            if (now - lastShotTime < Constants.Shooter.kSimFireRateSeconds) return;

            if (!Robot.Intake.obtainGamePieceSim()) return;

            var driveState = Robot.Drivetrain.getState();
            var turretFacing = Rotation2d.fromDegrees(Robot.Turret.getFieldRelativeAngleDegrees(driveState.Pose.getRotation()));

            lastShotTime = now;

            Translation2d turretFieldPos = TurretUtil.getTurretPose(driveState.Pose).getTranslation();

            // Apply calibration correction: divide by correction factor so the
            // vacuum-physics sim projectile lands where the real drag-affected ball does
            double rawExitVelocity = desiredRPS * Math.PI * Constants.Shooter.kWheelDiameterMeters;
            double distToTarget = turretFieldPos.getDistance(FieldConstants.getAllianceHub().toTranslation2d());
            double correction = TurretUtil.getCalibration().getCorrectionAt(distToTarget);
            double exitVelocity = rawExitVelocity / correction;

            // Convert robot-relative speeds to field-relative for projectile simulation
            double cos = driveState.Pose.getRotation().getCos();
            double sin = driveState.Pose.getRotation().getSin();
            ChassisSpeeds fieldSpeeds = new ChassisSpeeds(
                    driveState.Speeds.vxMetersPerSecond * cos - driveState.Speeds.vyMetersPerSecond * sin,
                    driveState.Speeds.vxMetersPerSecond * sin + driveState.Speeds.vyMetersPerSecond * cos,
                    driveState.Speeds.omegaRadiansPerSecond);

            GamePieceProjectile projectile = new RebuiltFuelOnFly(
                turretFieldPos,
                Translation2d.kZero,
                fieldSpeeds,
                turretFacing,
                Meters.of(Constants.Turret.kTurretOffsetZ),
                MetersPerSecond.of(exitVelocity),
                Degrees.of(Math.toDegrees(Constants.Shooter.kLaunchAngleRad)));

            SimulatedArena.getInstance().addGamePieceProjectile(projectile);
        }

    }

    @Override
    protected void periodicAny() {


    }

}