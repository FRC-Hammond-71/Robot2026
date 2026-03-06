package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;

public class CurrentDetection {
    private final double overcurrentThreshold;
    private final double requiredDurationSeconds;
    private double overcurrentStartTime = -1;
    
    public CurrentDetection(double overcurrentThreshold, double requiredDurationSeconds) {
        this.overcurrentThreshold = overcurrentThreshold;
        this.requiredDurationSeconds = requiredDurationSeconds;
    }

    public boolean isOvercurrent(double currentAmps) {
        if (currentAmps >= overcurrentThreshold) {
            if (overcurrentStartTime < 0) {
                overcurrentStartTime = Timer.getFPGATimestamp();
            }
            return (Timer.getFPGATimestamp() - overcurrentStartTime) >= requiredDurationSeconds;
        } else {
            overcurrentStartTime = -1;
            return false;
        }
    }

    public void reset() {
        overcurrentStartTime = -1;
    }
}
