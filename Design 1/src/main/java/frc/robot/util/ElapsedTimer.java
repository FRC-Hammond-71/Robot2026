package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;
import java.time.Duration;

public class ElapsedTimer
{
    public final Timer Timer;
    public final Duration Period;

    public ElapsedTimer(Duration period)
    {
        this.Timer = new Timer();
        this.Timer.stop();
        this.Period = period;
    }

    public boolean hasElapsed()
    {
        return this.Timer.hasElapsed(this.Period.getSeconds());
    }

    public boolean advancedIfElapsed()
    {
        return this.Timer.advanceIfElapsed(this.Period.getSeconds());
    }
}