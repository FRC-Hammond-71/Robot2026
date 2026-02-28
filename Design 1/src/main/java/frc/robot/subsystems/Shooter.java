package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {

    private final TalonFX m_motorA = new TalonFX(41);
    private final TalonFX m_motorB = new TalonFX(42);
    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0).withSlot(0);

    public Shooter() {
        TalonFXConfiguration cfgA = new TalonFXConfiguration();
        cfgA.CurrentLimits.StatorCurrentLimit = 40;
        cfgA.CurrentLimits.StatorCurrentLimitEnable = true;
        Slot0Configs pid = cfgA.Slot0;
        pid.kP = ShooterConstants.kP;
        pid.kI = ShooterConstants.kI;
        pid.kD = ShooterConstants.kD;
        pid.kV = ShooterConstants.kV;
        m_motorA.getConfigurator().apply(cfgA);

        // Motor B: same PID, NO hardware inversion — handled in software
        TalonFXConfiguration cfgB = new TalonFXConfiguration();
        cfgB.CurrentLimits.StatorCurrentLimit = 40;
        cfgB.CurrentLimits.StatorCurrentLimitEnable = true;
        Slot0Configs pidB = cfgB.Slot0;
        pidB.kP = ShooterConstants.kP;
        pidB.kI = ShooterConstants.kI;
        pidB.kD = ShooterConstants.kD;
        pidB.kV = ShooterConstants.kV;
        m_motorB.getConfigurator().apply(cfgB);
    }

    // --- Velocity control ---
    public void setVelocity(double rps) {
        
        double clamped = rps == 0 ? 0 : Math.max(ShooterConstants.kMinSpeedRPS, Math.min(ShooterConstants.kMaxSpeedRPS, rps)); 

        m_motorA.setControl(m_velocityRequest.withVelocity(-clamped));
        m_motorB.setControl(m_velocityRequest.withVelocity(clamped)); // software inversion
    }

    public void stop() {
        m_motorA.set(0);
        m_motorB.set(0);
    }

    // --- Ready check ---
    public boolean isAtSpeed(double targetRPS) {
        double velA = m_motorA.getVelocity().getValueAsDouble();
        double velB = m_motorB.getVelocity().getValueAsDouble();
        return Math.abs(velA - targetRPS)  <= ShooterConstants.kSpeedToleranceRPS
            && Math.abs(velB + targetRPS)  <= ShooterConstants.kSpeedToleranceRPS; // B is negative
    }

    /**
     * Calculates required flywheel RPS from horizontal distance using
     * projectile motion physics for a fixed 42-degree launch angle.
     *
     * Formula: v = sqrt( (g * d^2) / (cos^2(θ) * (2*d*tan(θ) - 2*h)) )
     * Then: RPS = v / (π * wheelDiameter)
     *
     * Requires tuning of kHeightDeltaMeters, kWheelDiameterMeters, kSlipFactor.
     */
    public double calculateTargetRPS(double distanceMeters) {
        double angle = ShooterConstants.kLaunchAngleRad;
        double h     = ShooterConstants.kHeightDeltaMeters;
        double d     = distanceMeters;
        double g     = ShooterConstants.kG;

        double numerator   = g * d * d;
        double denominator = Math.pow(Math.cos(angle), 2)
                             * (2.0 * d * Math.tan(angle) - 2.0 * h);

        // Guard: geometry unsolvable (target too close or below launcher plane)
        if (denominator <= 0) {
            SmartDashboard.putBoolean("Shooter/PhysicsValid", false);
            return ShooterConstants.kMinSpeedRPS;
        }

        double exitVelocity    = Math.sqrt(numerator / denominator);
        double adjustedVelocity = exitVelocity / ShooterConstants.kSlipFactor;
        double targetRPS       = adjustedVelocity / (Math.PI * ShooterConstants.kWheelDiameterMeters);

        SmartDashboard.putBoolean("Shooter/PhysicsValid",   true);
        SmartDashboard.putNumber("Shooter/ExitVelocity_ms", exitVelocity);

        return targetRPS;
    }

    // --- Distance from robot pose to hub center ---
    public double calculateDistance(Translation2d robotTranslation) {
        return robotTranslation.getDistance(ShooterConstants.kHubPosition);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter/VelocityA_RPS", m_motorA.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/VelocityB_RPS", m_motorB.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/CurrentA",      m_motorA.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/CurrentB",      m_motorB.getStatorCurrent().getValueAsDouble());
    }
}