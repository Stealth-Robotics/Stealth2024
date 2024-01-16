package frc.robot.subsystems.Vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PoseEstimationSystem extends SubsystemBase {

    private final String LEFT_CAMERA_NAME = "camera0";
    private final String RIGHT_CAMERA_NAME = "camera1";

    private final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT;

    private final Transform3d LEFT_CAMERA_ROBOT_TO_CAM_TRANSFORM_METERS = new Transform3d(
            new Translation3d(0, 0, 0),
            new Rotation3d(0, 0, 0));

    private final Transform3d RIGHT_CAMERA_ROBOT_TO_CAM_TRANSFORM_METERS = new Transform3d(
            new Translation3d(0, 0, 0),
            new Rotation3d(0, 0, 0));

    private final CameraSubsystem leftCamera;
    private final CameraSubsystem rightCamera;

    private Optional<EstimatedRobotPose> leftCameraPose;
    private Optional<EstimatedRobotPose> rightCameraPose;

    private double leftCameraTimestamp = 0.0;
    private double rightCameraTimestamp = 0.0;

    public PoseEstimationSystem() {
        try {
            APRIL_TAG_FIELD_LAYOUT = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
        } catch (Exception e) {
            throw new RuntimeException("Failed to load AprilTag field layout");
        }

        leftCamera = new CameraSubsystem(LEFT_CAMERA_NAME, APRIL_TAG_FIELD_LAYOUT,
                LEFT_CAMERA_ROBOT_TO_CAM_TRANSFORM_METERS);
        rightCamera = new CameraSubsystem(RIGHT_CAMERA_NAME, APRIL_TAG_FIELD_LAYOUT,
                RIGHT_CAMERA_ROBOT_TO_CAM_TRANSFORM_METERS);

    }

    private boolean getVisionEstimatePresent(Optional<EstimatedRobotPose> visionEstimate) {
        return visionEstimate.isPresent();
    }

    public boolean getLeftVisionEstimatePresent() {
        return getVisionEstimatePresent(leftCameraPose);
    }

    public boolean getRightVisionEstimatePresent() {
        return getVisionEstimatePresent(rightCameraPose);
    }

    private Pose2d getVisionEstimatePose2d(Optional<EstimatedRobotPose> visionEstimate) {
        return visionEstimate.get().estimatedPose.toPose2d();
    }

    public Pose2d getLeftVisionEstimatePose2d() {
        return getVisionEstimatePose2d(leftCameraPose);
    }

    public Pose2d getRightVisionEstimatePose2d() {
        return getVisionEstimatePose2d(rightCameraPose);
    }

    public double getLeftVisionEstimateTimestamp() {
        return leftCameraTimestamp;
    }

    public double getRightVisionEstimateTimestamp() {
        return rightCameraTimestamp;
    }


    @Override
    public void periodic() {
        leftCameraPose = leftCamera.getEstimatedGlobalPose();
        rightCameraPose = rightCamera.getEstimatedGlobalPose();

        if (getLeftVisionEstimatePresent()) {
            leftCameraTimestamp = leftCameraPose.get().timestampSeconds;

            
            // System.out.println("left present:" + getLeftVisionEstimatePresent() + ", timestamp: "
            //         + getLeftVisionEstimateTimestamp() + ", pose2d:" + getLeftVisionEstimatePose2d());
        }

        if (getRightVisionEstimatePresent()) {
            rightCameraTimestamp = rightCameraPose.get().timestampSeconds;


            // System.out.println("right present:" + getRightVisionEstimatePresent() + ", timestamp: "
            //         + getRightVisionEstimateTimestamp() + ", pose2d:" + getRightVisionEstimatePose2d());

        }
    }

}
