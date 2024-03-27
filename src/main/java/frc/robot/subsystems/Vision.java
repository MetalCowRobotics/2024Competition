package frc.robot.subsystems;

import frc.robot.Constants;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

    private PhotonCamera camera;
    private boolean aprilTagHasTargets;
    private PhotonTrackedTarget aprilTagBestTarget;
    private AprilTagFieldLayout aprilTagFieldLayout;
    private PhotonPoseEstimator photonPoseEstimator;
    private int fiducialID;
    private Transform3d robotToCam;
    private boolean visionStatus;

    public Vision() {
        camera = new PhotonCamera("MicrosoftLifeCamHD-3000");
        camera.setDriverMode(false);
        aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        robotToCam = Constants.VisionConstants.robotToCamTranslation;
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, robotToCam);
        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    public Optional<EstimatedRobotPose> getPoseEstimate() {
        if (visionStatus) {
            return photonPoseEstimator.update();
        }
        return Optional.empty();
    }

    public void setVisionStatus (boolean visionStatusRequest) {
        visionStatus = visionStatusRequest;
    }
}