package frc.robot.subsystems.Vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public class CameraSubsystem {

    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;
    private double lastTimestamp = 0.0;

    public CameraSubsystem(String cameraName, AprilTagFieldLayout fieldLayout, Transform3d robotTocamera) {
        camera = new PhotonCamera(cameraName);
        poseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, robotTocamera);
        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    public PhotonPipelineResult getLatestResult() {
        return camera.getLatestResult();
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        var visionEstimate = poseEstimator.update();
        double timestamp = camera.getLatestResult().getTimestampSeconds();
        boolean newResult = Math.abs(lastTimestamp - timestamp) > 1e-5;
        if(newResult) {
            lastTimestamp = timestamp;
        }
        return visionEstimate;
    }
    
}
