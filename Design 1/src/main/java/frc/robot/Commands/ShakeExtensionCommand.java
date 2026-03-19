package frc.robot.Commands;

import java.time.Duration;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.util.ElapsedTimer;

public class ShakeExtensionCommand extends Command {

    private final IntakeSubsystem intake;
    private final double speed;
    private final ElapsedTimer timer;

    public ShakeExtensionCommand(IntakeSubsystem intake, double speed, Duration interval) {
        this.intake = intake;
        this.speed = speed;
        this.timer = new ElapsedTimer(interval);
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        timer.Timer.restart();
        if (intake.isExtended) {
            intake.retract(speed);
        } else {
            intake.extend(speed);
        }
    }

    @Override
    public void execute() {
        if (timer.advancedIfElapsed()) {
            intake.isExtended = !intake.isExtended;
            if (intake.isExtended) {
                intake.extend(speed);
            } else {
                intake.retract(speed);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopExtension();
        timer.Timer.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
