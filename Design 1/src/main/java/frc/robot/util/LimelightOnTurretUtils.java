package frc.robot.util;

import edu.wpi.first.math.geometry.*;
import frc.robot.Constants;

public class LimelightOnTurretUtils {

    public static Pose2d getRobotPoseFromCameraPose(Pose2d cameraFieldPose2d, Rotation3d turretRotation3d) {

        Pose3d cameraFieldPose = new Pose3d(
            cameraFieldPose2d.getX(),
            cameraFieldPose2d.getY(),
            0.0,
            new Rotation3d(0, 0, cameraFieldPose2d.getRotation().getRadians())
        );

        // Turret rotation goes here so that plus() rotates turretToCamera.translation
        // by turretRotation3d, correctly placing the camera in the robot frame.
        Transform3d robotToTurret = new Transform3d(
            Constants.Turret.kTurretOffsetFromRobotCenter,
            turretRotation3d
        );

        // Fixed offset of camera from turret pivot, in the turret's local frame.
        Transform3d turretToCamera = new Transform3d(
            Constants.Vision.kLimelightOffsetFromTurretOffset.getTranslation(),
            new Rotation3d()
        );

        // plus() computes: translation = kTurretOffset + turretRotation.apply(kCameraOffset)
        Transform3d robotToCamera = robotToTurret.plus(turretToCamera);

        // transformBy handles field-frame conversion internally; no manual yaw rotation needed.
        Pose3d robotFieldPose = cameraFieldPose.transformBy(robotToCamera.inverse());

        return robotFieldPose.toPose2d();
    }
}