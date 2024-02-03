package frc.robot.subsystems;

import frc.robot.Constants;

import java.io.IOException;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

public class Vision {
    private PhotonCamera camera;
    private PhotonPipelineResult aprilTagResult;
    private boolean aprilTagHasTargets;
    private List<PhotonTrackedTarget> aprilTagTargets;
    private PhotonTrackedTarget aprilTagBestTarget;
    private AprilTagFieldLayout aprilTagFieldLayout;
    private PhotonPoseEstimator poseEstimator;
    private int fiducialID;
    private Transform3d robotToCam;
    private double aprilTagX, aprilTagY, aprilTagZAngle, aprilTagZ = -1;
    private Pose2d globalPoseEstimate = new Pose2d();
    private Transform3d fieldToCamera;

    public void AprilTagVision() {
        camera = new PhotonCamera("MicrosoftLifeCamHD-3000");
        aprilTagResult = new PhotonPipelineResult();
        aprilTagHasTargets = false;
        try {
            aprilTagFieldLayout = new AprilTagFieldLayout(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        robotToCam = new Transform3d(Constants.VisionConstants.robotToCamTranslation, Constants.VisionConstants.robotToCamRotation);
        poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera ,robotToCam);
    }

    public boolean hasTargets(){
        return this.aprilTagHasTargets;
    }

    public Pose2d getGlobalPoseEstimate() {
        return this.globalPoseEstimate;
    }

    public double getTimestampSeconds(){
        return this.aprilTagResult.getTimestampSeconds();
    }

    public Rotation2d getAprilTagGyroYaw() {
        return Rotation2d.fromRadians(this.aprilTagZAngle);
    }

    public int getID() {
        return this.fiducialID;
    }

    public void periodic() {
        aprilTagResult = camera.getLatestResult();
        aprilTagHasTargets = aprilTagResult.hasTargets();

        if (aprilTagHasTargets) {
            aprilTagTargets = aprilTagResult.getTargets();
            aprilTagBestTarget = aprilTagResult.getBestTarget();
            globalPoseEstimate = new Pose2d(fieldToCamera.getX(), fieldToCamera.getY(), new Rotation2d(fieldToCamera.getRotation().getX(), fieldToCamera.getRotation().getY()));

            fiducialID = aprilTagBestTarget.getFiducialId();
            aprilTagX = aprilTagBestTarget.getBestCameraToTarget().getX();
            aprilTagY = aprilTagBestTarget.getBestCameraToTarget().getY();
            aprilTagZ = aprilTagBestTarget.getBestCameraToTarget().getZ();
            aprilTagZAngle = aprilTagBestTarget.getBestCameraToTarget().getRotation().getAngle();
            if (aprilTagResult.getMultiTagResult().estimatedPose.isPresent){
                fieldToCamera = aprilTagResult.getMultiTagResult().estimatedPose.best;
            }   
        } 
        else {
            fiducialID = -1;
            aprilTagX = -1.0;
            aprilTagY = -1.0;
            aprilTagZ = -1.0;
            aprilTagZAngle = -1.0;
        }
    }
}
