package frc.robot.util;

import edu.wpi.first.math.geometry.*;
import frc.robot.Constants;

public class LimelightOnTurretUtils {

    public static Pose2d getRobotPoseFromCameraPose(Pose2d cameraFieldPose2d, Rotation3d turretRotation3d, Rotation2d robotYaw) {

        Pose3d cameraFieldPose = new Pose3d(
            cameraFieldPose2d.getX(),
            cameraFieldPose2d.getY(),
            0.0,
            new Rotation3d(0, 0, cameraFieldPose2d.getRotation().getRadians())
        );

        Rotation3d robotRotation3d = new Rotation3d(0, 0, robotYaw.getRadians());

        Transform3d robotToTurret = new Transform3d(
            Constants.Turret.kTurretOffsetFromRobotCenter,
            new Rotation3d()
        );

        Transform3d turretToCamera = new Transform3d(
            Constants.Vision.kLimelightOffsetFromTurretOffset.getTranslation(),
            turretRotation3d
        );

        Transform3d robotToCamera = robotToTurret.plus(turretToCamera);

        Transform3d robotToCameraWithYaw = new Transform3d(
            robotToCamera.getTranslation().rotateBy(robotRotation3d),
            robotRotation3d.plus(robotToCamera.getRotation())
        );

        Pose3d robotFieldPose = cameraFieldPose.transformBy(robotToCameraWithYaw.inverse());

        return robotFieldPose.toPose2d();
    }
}