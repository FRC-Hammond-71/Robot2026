package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import static edu.wpi.first.units.Units.Meters;

import java.time.Duration;
import java.util.Optional;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.SubsystemWithMapleSimSimulation;
import frc.robot.util.CurrentDetection;
import frc.robot.util.ElapsedTimer;

public class IntakeSubsystem extends SubsystemWithMapleSimSimulation {

    private final Optional<IntakeSimulation> Sim;

    // KrakenX60 — intake shaft
    private final TalonFX m_intakeMotor = new TalonFX(Constants.Intake.kIntakeMotorCanID);

    // NEO — linear extension (14t driving 48t)
    private final SparkMax m_extensionMotor = new SparkMax(Constants.Intake.kExtensionMotorCanID, MotorType.kBrushless);

    private final CurrentDetection extensionCurrentDetection = new CurrentDetection(
            Constants.Intake.kExtensionStallThreshold, Constants.Intake.kExtensionStallDurationSeconds);
    public boolean isExtended = false; // true = currently in extended position
    private boolean isExtensionMoving = false; // true = motor actively running to extend/retract
    private double extensionMoveStartTime = 0; // FPGA timestamp when extension movement began

    public IntakeSubsystem(Robot robotInstance) {

        super(robotInstance);

        if (RobotBase.isSimulation()) {

            Sim = Optional.of(IntakeSimulation.OverTheBumperIntake(
                "Fuel",
                robotInstance.Drivetrain.getMapleSimSwerveDrivetrain(),
                Meters.of(Constants.Simulation.kIntakeWidthMeters),
                Meters.of(Constants.Simulation.kIntakeExtensionMeters),
                IntakeSimulation.IntakeSide.FRONT,
                Constants.Intake.kMaxGamePieces));

            // SimulatedArena.getInstance().addDriveTrainSimulation(robotInstance.Drivetrain.getMapleSimSwerveDrivetrain());

        } else {

            Sim = Optional.empty();

        }

        // KrakenX60 config
        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
        intakeConfig.CurrentLimits.StatorCurrentLimit = Constants.Intake.kStatorCurrentLimit;
        intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        m_intakeMotor.getConfigurator().apply(intakeConfig);

        // NEO extension config — gear ratio applied as position conversion factor
        SparkMaxConfig extensionConfig = new SparkMaxConfig();
        extensionConfig.smartCurrentLimit(Constants.Intake.kExtensionCurrentLimit);
        extensionConfig.encoder.positionConversionFactor(1.0 / Constants.Intake.kExtensionGearRatio);
        m_extensionMotor.configure(extensionConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public int getGamePiecesHeldSimm() {
        return this.Sim.isPresent() ? Sim.get().getGamePiecesAmount() : 0;
    }

    public boolean obtainGamePieceSim()
    {
        return this.Sim.isPresent() && Sim.get().obtainGamePieceFromIntake();
    }

    @Override
    public void periodic() {
        double extensionCurrent = m_extensionMotor.getOutputCurrent();
        SmartDashboard.putNumber("Intake/Extension/Amperage", extensionCurrent);
        SmartDashboard.putBoolean("Intake/Extension/IsExtended", isExtended);
        SmartDashboard.putBoolean("Intake/Extension/IsMoving", isExtensionMoving);
            // SmartDashboard.putNumber("Intake/GamePiecesHeld", simManager.getGamePiecesHeld());

        if (isExtensionMoving) {
            boolean timedOut = Timer.getFPGATimestamp() - extensionMoveStartTime >= 1.5;
            boolean done = timedOut || (!RobotBase.isSimulation() && extensionCurrentDetection.isOvercurrent(extensionCurrent));
            if (done) {
                m_extensionMotor.set(0);
                isExtensionMoving = false;
            } else {
                // Continuously drive — SparkMax motor safety requires periodic updates
                m_extensionMotor.set(isExtended ? 0.35 : -0.35);
            }
        }

        // Intake motor: run while extending/extended, run full while retracting
        if (isExtensionMoving && !isExtended) {
            intake(1.0);
        } else if (isExtended) {
            intake(Constants.Intake.kHoldSpeed);
        } else {
            stop();
        }
    }

    public void intake(double speed) {
        m_intakeMotor.set(speed);

        if (this.Sim.isPresent()) Sim.get().startIntake();
    }

    public void stop() {
        m_intakeMotor.set(0);

        if (this.Sim.isPresent()) Sim.get().stopIntake();
    }

    public Command intakeCommand(double speed) {
        return Commands.runEnd(() -> intake(speed), this::stop, this).withName("Intake");
    }

    /**
     * Toggles the extension: if retracted, starts extending; if extended, starts
     * retracting.
     * The motor stops automatically in periodic() when a 40A current spike is
     * detected.
     */
    public void toggleExtension() {
        isExtended = !isExtended;
        isExtensionMoving = true;
        extensionCurrentDetection.reset();
        extensionMoveStartTime = Timer.getFPGATimestamp();
    }

    public Command toggleExtensionCommand() {
        return Commands.runOnce(this::toggleExtension).withName("ToggleExtension");
    }

    public void extend(double speed) {
        m_extensionMotor.set(speed);
    }

    public void retract(double speed) {
        m_extensionMotor.set(-speed);
    }

    public void stopExtension() {
        m_extensionMotor.set(0);
        if (!isExtended) {
            stop();
        }
    }

    public double getExtensionPosition() {
        return m_extensionMotor.getEncoder().getPosition(); // output shaft rotations
    }

    public Command extendCommand(double speed) {
        return Commands.runEnd(() -> extend(speed), this::stopExtension, this).withName("Intake Extend");
    }

    public Command retractCommand(double speed) {
        return Commands.runEnd(() -> retract(speed), this::stopExtension, this).withName("Intake Retract");
    }

    @Override
    protected void periodicReal() {
        

    }

    @Override
    protected void periodicSimulated() {
       

    }

    @Override
    protected void periodicAny() {
       

    }

}
