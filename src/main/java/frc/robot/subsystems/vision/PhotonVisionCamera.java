package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVisionCamera extends SubsystemBase{
    PhotonCamera camera0;
    PhotonCamera camera1;
    AprilTagFieldLayout aprilTagFieldLayout;


    private final double kCamera0OffsetX = 0.0;
    private final double kCamera0OffsetY = 0.0;
    private final double kCamera0OffsetZ = 0.0;
    private final double kCamera0OffsetYaw = 0.0;

    private final double kCamera1OffsetX = 0.0;
    private final double kCamera1OffsetY = 0.0;
    private final double kCamera1OffsetZ = 0.0;
    private final double kCamera1OffsetYaw = 0.0;

    private final Rotation3d kCamera0OffsetYawRotation = new Rotation3d(kCamera0OffsetYaw, 0.0, 0.0);
    private final Rotation3d kCamera1OffsetYawRotation = new Rotation3d(kCamera1OffsetYaw, 0.0, 0.0);


    private final Translation3d kCamera0OffsetTranslation = new Translation3d(kCamera0OffsetX, kCamera0OffsetY, kCamera0OffsetZ);
    private final Translation3d kCamera1OffsetTranslation = new Translation3d(kCamera1OffsetX, kCamera1OffsetY, kCamera1OffsetZ);



    private final Transform3d kCamera0Offset = new Transform3d(kCamera0OffsetTranslation, kCamera0OffsetYawRotation);
    private final Transform3d kCamera1Offset = new Transform3d(kCamera1OffsetTranslation, kCamera1OffsetYawRotation);    

    public PhotonVisionCamera(){
        PhotonCamera camera0 = new PhotonCamera("camera0");

        PhotonCamera camera1 = new PhotonCamera("camera1");

        try{
            AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);

        }
        catch(Exception e){
            throw new RuntimeException(e);
        }
    }

    


    public List<PhotonTrackedTarget> getCameraTargets(){
        var result0 = camera0.getLatestResult();
        var result1 = camera1.getLatestResult();

        List<PhotonTrackedTarget> targets = new ArrayList<>();

        if(result0.hasTargets()){
            targets.set(0, result0.getBestTarget());
        }
        else{
            targets.set(0, null);
        }

        if(result1.hasTargets()){
            targets.set(1, result1.getBestTarget());
        }
        else{
            targets.set(1, null);
        }

        return targets;
    }
    

    public Optional<Pose3d> getFieldPositionFromTargets(List<PhotonTrackedTarget> targets){
        Optional<Pose3d> fieldPosition = Optional.empty();
        Pose3d camera0FieldPose = new Pose3d();
        Pose3d camera1FieldPose = new Pose3d();
        if(targets.get(0) != null){
            Optional<Pose3d> aprilTagPose = aprilTagFieldLayout.getTagPose(targets.get(0).getFiducialId());
            camera0FieldPose = PhotonUtils.estimateFieldToRobotAprilTag(targets.get(0).getBestCameraToTarget(), aprilTagPose.get(), kCamera0Offset);
            
        }
        if(targets.get(1) != null){
            Optional<Pose3d> aprilTagPose = aprilTagFieldLayout.getTagPose(targets.get(1).getFiducialId());
            camera1FieldPose = PhotonUtils.estimateFieldToRobotAprilTag(targets.get(1).getBestCameraToTarget(), aprilTagPose.get(), kCamera1Offset);
            
        }
        if(targets.get(0) != null && targets.get(1) != null){
            Pose3d averagePose = new Pose3d();
            averagePose = camera0FieldPose.plus(new Transform3d(camera1FieldPose.getTranslation(), camera1FieldPose.getRotation()));
            averagePose = averagePose.times(0.5);
            fieldPosition = Optional.of(averagePose);
        }
        else if(targets.get(0) != null){
            fieldPosition = Optional.of(camera0FieldPose);
        }
        else if(targets.get(1) != null){
            fieldPosition = Optional.of(camera1FieldPose);
        }

        return fieldPosition;
    }


    
}
