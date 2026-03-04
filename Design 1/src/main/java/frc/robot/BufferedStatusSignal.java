package frc.robot;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Timestamp;
import com.ctre.phoenix6.Timestamp.TimestampSource;

/**
 * Stores measurements from a CTRe Status Singal for a certain amount of time.
 */
public class BufferedStatusSignal<T>
{
    private final double measurementPeriodMs;
    private final StatusSignal<T> statusSignal;
    private final TimestampSource requiredTimestampSource;

    private final double[] valuesAt;

    private long firstTimestampMs = -1;

    public BufferedStatusSignal(
        StatusSignal<T> statusSignal,
        TimestampSource requiredSource,
        int measurementPeriodMs,
        int toStoreMs)
    {
        this.statusSignal = statusSignal;
        this.requiredTimestampSource = requiredSource;
        this.measurementPeriodMs = measurementPeriodMs;

        int measurementsToStore = (int)(toStoreMs / this.measurementPeriodMs) + 1;

        this.valuesAt = new double[measurementsToStore];
    }

    public void periodic()
    {
        double val = statusSignal.getValueAsDouble();
        long timestampMs = (long)(statusSignal.getTimestamp().getTime() * 1000.0);

        if (firstTimestampMs < 0)
        {
            firstTimestampMs = timestampMs;
        }

        long elapsedMs = timestampMs - firstTimestampMs;
        int index = (int)(elapsedMs / measurementPeriodMs) % valuesAt.length;

        if (index < 0)
        {
            index += valuesAt.length;
        }

        valuesAt[index] = val;
    }

    public double getValueAt(Timestamp atTime)
    {
        if (atTime.getSource() != requiredTimestampSource)
        {
            throw new IllegalArgumentException("atTime must use TimestampSource " + requiredTimestampSource + ", but was " + atTime.getSource());
        }

        if (firstTimestampMs < 0)
        {
            return statusSignal.getValueAsDouble();
        }

        long targetTimeMs = (long)(atTime.getTime() * 1000.0);
        return interpolate(targetTimeMs);
    }

    public double getValueAt(double timeSeconds, TimestampSource source)
    {
        if (source != requiredTimestampSource)
        {
            throw new IllegalArgumentException("source must be " + requiredTimestampSource + ", but was " + source);
        }

        if (firstTimestampMs < 0)
        {
            return statusSignal.getValueAsDouble();
        }

        long targetTimeMs = (long)(timeSeconds * 1000.0);
        return interpolate(targetTimeMs);
    }

        /**
     * Linearly interpolates between the two buffer slots bracketing targetTimeMs.
     * Clamps to the earliest stored value if targetTimeMs is before firstTimestampMs.
     */
    private double interpolate(long targetTimeMs)
    {
        double exactIndex = (targetTimeMs - firstTimestampMs) / measurementPeriodMs;

        // Clamp to earliest available value if querying before buffer start
        if (exactIndex < 0) exactIndex = 0;

        int lowerIdx = (int) Math.floor(exactIndex);
        int upperIdx = lowerIdx + 1;
        double fraction = exactIndex - lowerIdx;

        int i1 = ((lowerIdx % valuesAt.length) + valuesAt.length) % valuesAt.length;
        int i2 = ((upperIdx % valuesAt.length) + valuesAt.length) % valuesAt.length;

        return valuesAt[i1] + fraction * (valuesAt[i2] - valuesAt[i1]);
    }
}
