package frc.robot.util;

public class CurrentDetection 
{
    private double OvercurrentThreshold;

    public CurrentDetection(double overcurrentThreshold) 
    {
        this.OvercurrentThreshold = overcurrentThreshold;
    }

    public boolean isOvercurrent(double currentCurrent)
    {
        return currentCurrent >= OvercurrentThreshold;
    }
}
