package frc.robot.subsystems;

import frc.robot.Constants;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

    public double getYawOfBestTarget() {
        if (camera.getLatestResult().hasTargets()) {
            return camera.getLatestResult().getBestTarget().getYaw();
        } else {
            return 0;
        }
    }
}