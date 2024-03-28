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

    public Vision() {
        camera = new PhotonCamera("MicrosoftLifeCamHD-3000");
        camera.setDriverMode(false);
        cameraPipelineResult = camera.getLatestResult();
        aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        // robotToCam = Constants.VisionConstants.robotToCamTranslation;
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, new Transform3d());
        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    public Optional<EstimatedRobotPose> getPoseEstimate() {
        if (cameraPipelineResult.hasTargets()) {
            SmartDashboard.putBoolean("HasTarget", true);
            return photonPoseEstimator.update();
        } else {
            SmartDashboard.putBoolean("HasTarget", false);
            return Optional.empty();
        }
    }

    public int getBestID() {
        if (cameraPipelineResult.hasTargets()) {
            int bestID = cameraPipelineResult.getBestTarget().getFiducialId();
            return bestID;
        } else {
            return -1;
        }
    }
}