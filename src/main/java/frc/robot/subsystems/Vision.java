package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

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

    public Vision() {
        camera = new PhotonCamera("MicrosoftLifeCamHD-3000");
        camera.setDriverMode(false);
        cameraPipelineResult = camera.getLatestResult();
        aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        robotToCam = Constants.VisionConstants.robotToCamTranslation;
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, camera, robotToCam);
        bestYaw = 0;
    }

    public Optional<EstimatedRobotPose> getPoseEstimate() {
        if (cameraPipelineResult.hasTargets()) {
            return photonPoseEstimator.update();
        } else {
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
        if (camera.getLatestResult().hasTargets()){
            if((camera.getLatestResult().getBestTarget().getFiducialId() == 4) || (camera.getLatestResult().getBestTarget().getFiducialId() == 7) || (camera.getLatestResult().getBestTarget().getFiducialId() == 8) ||(camera.getLatestResult().getBestTarget().getFiducialId() == 3)){
                return camera.getLatestResult().getBestTarget().getBestCameraToTarget().getTranslation();
            }
            return new Translation3d();
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



    public double getYawOfBestTarget() {
        if (camera.getLatestResult().hasTargets()) {
            return camera.getLatestResult().getBestTarget().getYaw();
        } else {
            return 0;
        }
    }
}