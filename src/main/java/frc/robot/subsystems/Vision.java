package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {

    private PhotonCamera camera;
    private PhotonPipelineResult cameraPipelineResult;
    private AprilTagFieldLayout aprilTagFieldLayout;
    private PhotonPoseEstimator photonPoseEstimator;
    private Transform3d robotToCam;
    public double bestYaw;
    private Translation3d visionPlaceholder;

    public Vision() {
        camera = new PhotonCamera("2024Crescendo");
        camera.setDriverMode(false);
        cameraPipelineResult = camera.getLatestResult();
        aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        robotToCam = Constants.VisionConstants.robotToCamTranslation;
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, camera, robotToCam);
        bestYaw = 0;
    }


    @Override
    public void periodic() {
        cameraPipelineResult = camera.getLatestResult();
        
        SmartDashboard.putBoolean("Camera Connected", camera.isConnected());
        SmartDashboard.putBoolean("Has Targets", cameraPipelineResult.hasTargets());
        if (cameraPipelineResult.hasTargets()) {
            SmartDashboard.putNumber("Target ID", cameraPipelineResult.getBestTarget().getFiducialId());
            SmartDashboard.putNumber("Target Yaw", cameraPipelineResult.getBestTarget().getYaw());
        }
    }

    public Optional<EstimatedRobotPose> getPoseEstimate() {
        if (cameraPipelineResult.hasTargets()) {
            SmartDashboard.putBoolean("Bruh", true);
            return photonPoseEstimator.update();
        } else {
            SmartDashboard.putBoolean("Bruh", false);
            return Optional.empty();
        }
    }

    public int getBestID() {
        if (cameraPipelineResult.hasTargets()) {
            int bestID = cameraPipelineResult.getBestTarget().getFiducialId();
            SmartDashboard.putNumber("BestID", bestID);
            return bestID;
        } else {
            SmartDashboard.putNumber("BestID", -1);
            return -1;
        }
    }

    /*
     * Speaker Tag IDs
     * Red Alliance: 4
     * Blue Alliance: 7
     */
    public Translation3d getDistFromScoringTag(){
        PhotonTrackedTarget x = camera.getLatestResult().getBestTarget();
        //if (camera.getLatestResult().hasTargets()){
        if (x != null) {
            if((x.getFiducialId() == 4) || (x.getFiducialId() == 7) || (x.getFiducialId() == 8) ||(x.getFiducialId() == 3)){
                visionPlaceholder = x.getBestCameraToTarget().getTranslation();
                return visionPlaceholder;
            }
            return visionPlaceholder;
        }
        return new Translation3d();
    }

    public double getTotalDist(){
        Translation3d allDistances = getDistFromScoringTag();
        double xcomp = allDistances.getX();
        double ycomp = allDistances.getY();
        return Math.sqrt((xcomp*xcomp)+(ycomp*ycomp));
    }

    public Transform3d getCamToTag(){
        if (camera.getLatestResult().hasTargets()) {
            return camera.getLatestResult().getBestTarget().getBestCameraToTarget();
        } else {
            return new Transform3d();
        }
    }

    public double getTotalXDist(){
        Translation3d allDistances = getDistFromScoringTag();
        return allDistances.getX();
    }

    public double getTotalYDist(){
        Translation3d allDistances = getDistFromScoringTag();
        return allDistances.getY();
    }



    public double getYawOfBestTarget() {
        var result = camera.getLatestResult();
        if (result.hasTargets()) {
            return result.getBestTarget().getYaw();
        }
        return 0.0;
    }

    public boolean hasTargets() {
        return camera.getLatestResult().hasTargets();
    }

    public int getBestTargetID() {
        var result = camera.getLatestResult();
        if (result.hasTargets()) {
            return result.getBestTarget().getFiducialId();
        }
        return -1;
    }
}