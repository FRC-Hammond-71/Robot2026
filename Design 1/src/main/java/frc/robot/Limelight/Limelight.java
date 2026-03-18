package frc.robot.Limelight;

import java.util.ArrayDeque;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Limelight.LimelightHelpers.PoseEstimate;

public class Limelight {

    protected static final Map<String, Limelight> REGISTERED_LIMELIGHTS = new HashMap<>();

    private static final int SAMPLE_SIZE = 4;
    private static final double DISPLACEMENT_ERROR_MARGIN = 0.6096;
    private static final double MAX_ROBOT_SPEED = 4.7244;
    private static final double SPEED_MARGIN_MULTIPLIER = 1.2;
    private static final double FAR_TAG_DISTANCE_METERS = 4.0;
    private static final double STALE_THRESHOLD_SECONDS = 0.5;
    private static final int MIN_TAGS_FOR_STABLE_POSE = 2;

    public static void registerDevice(String name) {
        REGISTERED_LIMELIGHTS.putIfAbsent(name, new Limelight(name));
    }

    public static Optional<Limelight> useDevice(String name) {
        return Optional.ofNullable(REGISTERED_LIMELIGHTS.get(name));
    }

    public final String name;

    private final TranslationMedianFilter translationFilter = new TranslationMedianFilter(SAMPLE_SIZE);

    private double lastVisionTimestamp = -1.0;
    private double lastAcceptedTimestamp = -1.0;
    private Pose2d lastRawVisionPose;
    private PoseEstimate lastPoseEstimate;
    private int acceptedStableSamples = 0;

    private Limelight(String name) {
        this.name = name;
        LimelightHelpers.SetIMUMode(this.name, 0);
    }

    private PoseEstimate getLatestEstimate() {
        PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(this.name);
        lastPoseEstimate = estimate;
        return estimate;
    }

    private static boolean isValidEstimate(PoseEstimate estimate) {
        return estimate != null && estimate.pose != null;
    }

    private static boolean isFinitePose(Pose2d pose, double timestamp) {
        return Double.isFinite(pose.getX())
                && Double.isFinite(pose.getY())
                && Double.isFinite(pose.getRotation().getRadians())
                && Double.isFinite(timestamp);
    }

    private void resetTrackingState() {
        lastRawVisionPose = null;
        lastVisionTimestamp = -1.0;
        acceptedStableSamples = 0;
        translationFilter.reset();
    }

    public Optional<Pose2d> getRawEstimatedPose() {
        PoseEstimate estimate = getLatestEstimate();
        return isValidEstimate(estimate) ? Optional.of(estimate.pose) : Optional.empty();
    }

    public Optional<Pose2d> getMegaTagEstimatedPose(int minTagCount) {
        PoseEstimate estimate = getLatestEstimate();

        if (!isValidEstimate(estimate) || estimate.tagCount < minTagCount) {
            return Optional.empty();
        }

        return Optional.of(estimate.pose);
    }

    public double getLastAvgTagDist() {
        return lastPoseEstimate != null ? lastPoseEstimate.avgTagDist : Double.MAX_VALUE;
    }

    public double getLastTimestampSeconds() {
        return lastAcceptedTimestamp;
    }

    public Optional<Pose2d> getLastRawPose() {
        return lastPoseEstimate != null && lastPoseEstimate.pose != null
                ? Optional.of(lastPoseEstimate.pose)
                : Optional.empty();
    }

    public double getLatencyInSeconds() {
        return (LimelightHelpers.getLatency_Capture(this.name)
                + LimelightHelpers.getLatency_Pipeline(this.name)) / 1000.0;
    }

    public void resetPose(Pose2d initialPose) {
        LimelightHelpers.SetRobotOrientation(
                this.name,
                initialPose.getRotation().getDegrees(),
                0, 0, 0, 0, 0
        );

        lastRawVisionPose = initialPose;
        lastVisionTimestamp = -1.0;
        lastPoseEstimate = null;
        acceptedStableSamples = 0;
        translationFilter.reset();
    }

    /**
     * Get a filtered pose estimate using MegaTag 1.
     * Uses the actual robot speed for tighter displacement rejection.
     *
     * @param robotSpeedMps current robot linear speed in m/s from drivetrain
     */
    public Optional<Pose2d> getStableEstimatedPose(double robotSpeedMps) {
        PoseEstimate estimate = getLatestEstimate();

        if (!isValidEstimate(estimate)) {
            return Optional.empty();
        }

        if (estimate.tagCount < MIN_TAGS_FOR_STABLE_POSE || estimate.avgTagDist > FAR_TAG_DISTANCE_METERS) {
            return Optional.empty();
        }

        Pose2d rawPose = estimate.pose;
        double timestamp = estimate.timestampSeconds;

        if (!isFinitePose(rawPose, timestamp)) {
            return Optional.empty();
        }

        if (lastRawVisionPose != null && lastVisionTimestamp >= 0.0) {
            double rawDt = timestamp - lastVisionTimestamp;

            if (rawDt <= 0.0) {
                return Optional.empty();
            }

            if (rawDt > STALE_THRESHOLD_SECONDS) {
                resetTrackingState();
            } else {
                double displacement = lastRawVisionPose.getTranslation().getDistance(rawPose.getTranslation());
                // Use actual speed with margin, capped at max robot speed
                double effectiveSpeed = Math.min(robotSpeedMps * SPEED_MARGIN_MULTIPLIER, MAX_ROBOT_SPEED);
                double maxAllowed = (effectiveSpeed * rawDt) + DISPLACEMENT_ERROR_MARGIN;

                if (displacement > maxAllowed) {
                    return Optional.empty();
                }
            }
        }

        Translation2d filteredTranslation = translationFilter.calculate(rawPose.getTranslation());

        lastRawVisionPose = rawPose;
        lastVisionTimestamp = timestamp;
        lastAcceptedTimestamp = timestamp;
        acceptedStableSamples++;

        if (acceptedStableSamples < SAMPLE_SIZE) {
            return Optional.empty();
        }

        return Optional.of(new Pose2d(filteredTranslation, rawPose.getRotation()));
    }

    private static class TranslationMedianFilter {
        private final int size;
        private final ArrayDeque<Translation2d> samples;

        TranslationMedianFilter(int size) {
            this.size = size;
            this.samples = new ArrayDeque<>(size);
        }

        Translation2d calculate(Translation2d sample) {
            if (samples.size() == size) {
                samples.removeFirst();
            }

            samples.addLast(sample);

            double[] xs = new double[samples.size()];
            double[] ys = new double[samples.size()];
            int i = 0;

            for (Translation2d translation : samples) {
                xs[i] = translation.getX();
                ys[i] = translation.getY();
                i++;
            }

            return new Translation2d(median(xs), median(ys));
        }

        void reset() {
            samples.clear();
        }

        private static double median(double[] values) {
            double[] sorted = values.clone();
            Arrays.sort(sorted);
            int mid = sorted.length / 2;

            if (sorted.length % 2 == 0) {
                return (sorted[mid - 1] + sorted[mid]) / 2.0;
            }

            return sorted[mid];
        }
    }
}
