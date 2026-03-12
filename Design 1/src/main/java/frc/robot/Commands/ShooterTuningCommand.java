package frc.robot.Commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.TurretSubsystem;

// https://github.com/GROTTAKE/PNW_Ri3D_2026/blob/main/WPILIB/src/main/java/frc/robot/commands/LauncherTuningCommand.java
public class ShooterTuningCommand extends Command {

    private AngularVelocity current, max, interval;
    private final ShooterSubsystem shooter;
    private final SpindexerSubsystem spindexer;
    private final Supplier<Boolean> doNextLevel, doPrevLevel, doShoot;

    private boolean spinning = false;
    private final Timer spinTimer = new Timer();
    private static final double SPIN_DURATION_SECONDS = 2.0;

    // Edge detection: only trigger on press→release (falling edge)
    private boolean prevNext = false;
    private boolean prevPrev = false;
    private boolean prevShoot = false;

    public ShooterTuningCommand(
        ShooterSubsystem shooter,
        SpindexerSubsystem spindexer,
        AngularVelocity min,
        AngularVelocity max,
        AngularVelocity interval,
        Supplier<Boolean> doNextLevel,
        Supplier<Boolean> doPrevLevel,
        Supplier<Boolean> doShoot
    ) {
        this.current = min;
        this.max = max;
        this.interval = interval;
        this.shooter = shooter;
        this.spindexer = spindexer;
        this.doNextLevel = doNextLevel;
        this.doPrevLevel = doPrevLevel;
        this.doShoot = doShoot;

        addRequirements(shooter, spindexer);
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("ShooterTuning/CurrentRPS", current.in(RotationsPerSecond));
        SmartDashboard.putNumber("ShooterTuning/IntervalRPS", interval.in(RotationsPerSecond));
        SmartDashboard.putBoolean("ShooterTuning/Spinning", spinning);

        if (spinning) {
            if (spinTimer.hasElapsed(SPIN_DURATION_SECONDS)) {
                spinning = false;
                spinTimer.stop();
                spinTimer.reset();
                shooter.stop();
                spindexer.stop();
            }
            return;
        }

        // Read current button states
        boolean curNext = doNextLevel.get();
        boolean curPrev = doPrevLevel.get();
        boolean curShoot = doShoot.get();

        // Detect falling edge (was pressed, now released)
        boolean nextReleased = prevNext && !curNext;
        boolean prevReleased = prevPrev && !curPrev;
        boolean shootReleased = prevShoot && !curShoot;

        // Save for next cycle
        prevNext = curNext;
        prevPrev = curPrev;
        prevShoot = curShoot;

        if (shootReleased) {
            spinning = true;
            spinTimer.restart();

            spindexer.clockwise(0.5);
            shooter.setVelocity(current.in(RotationsPerSecond));
        } else if (nextReleased) {
            current = current.plus(interval);
        } else if (prevReleased) {
            current = current.minus(interval);
        }
    }

    @Override
    public boolean isFinished() {
        return current.gte(max);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        spinning = false;
        spinTimer.stop();
        spinTimer.reset();
    }
}
