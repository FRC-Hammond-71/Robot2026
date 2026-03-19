package frc.robot.util;

import edu.wpi.first.math.geometry.*;
import frc.robot.Constants;

public class LimelightOnTurretUtils {

    /** All intermediate values from the camera→robot transform, for debugging. */
    public record TransformDebug(
        double camFromTurretX, double camFromTurretY,
        double camInRobotX, double camInRobotY,
        double robotHeadingRad,
        Pose2d robotPose
    ) {}

    /**
     * Converts a camera field pose to a robot field pose using 2D math.
     *
     * @param cameraFieldPose Camera's field position (XY from MegaTag) with
     *                        corrected heading (pigeon + turret angle)
     * @param turretAngleRad  Turret angle in radians (CCW+, 0 = forward)
     * @return Robot center's field pose
     */
    public static Pose2d getRobotPoseFromCameraPose(Pose2d cameraFieldPose, double turretAngleRad) {
        return getRobotPoseFromCameraPoseDebug(cameraFieldPose, turretAngleRad).robotPose;
    }

    /**
     * Same transform as {@link #getRobotPoseFromCameraPose}, but returns all
     * intermediate values for telemetry debugging.
     */
    public static TransformDebug getRobotPoseFromCameraPoseDebug(Pose2d cameraFieldPose, double turretAngleRad) {

        // Camera XY offset from turret pivot (in turret-local frame)
        double camFromTurretX = Constants.Vision.kCamFromTurretX;
        double camFromTurretY = Constants.Vision.kCamFromTurretY;

        // Rotate camera offset by turret angle, add turret pivot offset → camera in robot frame
        double cos = Math.cos(turretAngleRad);
        double sin = Math.sin(turretAngleRad);
        double camInRobotX = Constants.Turret.kTurretOffsetX + cos * camFromTurretX - sin * camFromTurretY;
        double camInRobotY = Constants.Turret.kTurretOffsetY + sin * camFromTurretX + cos * camFromTurretY;

        // Robot heading = camera heading - turret angle
        double robotHeadingRad = cameraFieldPose.getRotation().getRadians() - turretAngleRad;

        // Robot field position = camera field position - R(robotHeading) * camInRobot
        double cosH = Math.cos(robotHeadingRad);
        double sinH = Math.sin(robotHeadingRad);
        double robotX = cameraFieldPose.getX() - (cosH * camInRobotX - sinH * camInRobotY);
        double robotY = cameraFieldPose.getY() - (sinH * camInRobotX + cosH * camInRobotY);

        Pose2d robotPose = new Pose2d(robotX, robotY, new Rotation2d(robotHeadingRad));
        return new TransformDebug(camFromTurretX, camFromTurretY, camInRobotX, camInRobotY, robotHeadingRad, robotPose);
    }
}
