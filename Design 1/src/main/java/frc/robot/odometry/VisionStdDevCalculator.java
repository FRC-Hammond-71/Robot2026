package frc.robot.odometry;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants;

// Scales vision std devs by tag count and distance. More tags + closer = trust more.
public class VisionStdDevCalculator {

    public static Matrix<N3, N1> calculate(int tagCount, double avgTagDistM) {
        if (tagCount <= 0) {
            return Constants.Odometry.kDefaultVisionStdDevs;
        }

        double xyStdDev = Constants.Odometry.kBaseVisionStdDev
            + Constants.Odometry.kDistanceScaleFactor * avgTagDistM;

        if (tagCount == 1) {
            xyStdDev *= Constants.Odometry.kSingleTagMultiplier;
        }

        xyStdDev = Math.max(Constants.Odometry.kMinVisionStdDev,
            Math.min(Constants.Odometry.kMaxVisionStdDev, xyStdDev));

        return VecBuilder.fill(xyStdDev, xyStdDev, 1e9);
    }
}
