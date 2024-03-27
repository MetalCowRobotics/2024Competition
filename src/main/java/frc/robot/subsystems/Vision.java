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
    private PhotonPipelineResult aprilTagResult;
    private boolean aprilTagHasTargets;
    private PhotonTrackedTarget aprilTagBestTarget;
    private AprilTagFieldLayout aprilTagFieldLayout;
    private PhotonPoseEstimator photonPoseEstimator;
    private int fiducialID;
    private Transform3d robotToCam;

    private double time;
    private double photonX;
    private double photonY;

    public Vision() {
        camera = new PhotonCamera("MicrosoftLifeCamHD-3000");
        aprilTagResult = new PhotonPipelineResult();
        aprilTagHasTargets = false;
        // aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        // robotToCam = Constants.VisionConstants.robotToCamTranslation;
        // photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, robotToCam);
        // photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        
    }

    public void periodic(){
        aprilTagResult = camera.getLatestResult();
        aprilTagHasTargets = aprilTagResult.hasTargets();

        // Optional<EstimatedRobotPose> optionalEstimatedPoseRight = photonPoseEstimator.update();
        //     if (optionalEstimatedPoseRight.isPresent()) {
        //         EstimatedRobotPose estimatedPose = optionalEstimatedPoseRight.get();
        //         SmartDashboard.putNumber("Vision X Pose", estimatedPose.estimatedPose.getX());
        //         SmartDashboard.putBoolean("We Failing", false);
        //     }
        //     else {
        //         SmartDashboard.putBoolean("We Failing", true);
        //     }

        if (aprilTagHasTargets) {
            aprilTagBestTarget = aprilTagResult.getBestTarget();
            fiducialID = aprilTagBestTarget.getFiducialId();

            Transform3d targetPose = aprilTagBestTarget.getBestCameraToTarget();
            photonX = targetPose.getX();
            photonY = targetPose.getY();
            time = aprilTagResult.getTimestampSeconds();
        } else {
            fiducialID = -1;
        }
    }

    public boolean hasTargets() {
        return this.aprilTagHasTargets;
    }

    // public Optional<EstimatedRobotPose> returnEstimatedRobotPose() {
    //     return photonPoseEstimator.update(aprilTagResult);
    // }

    public double returnPhotonX() {
        return photonX;
    }

    public double returnPhotonY() {
        return photonY;
    }

    public double returnTime() {
        return time;
    }

    // public double getPhotonTimestampSeconds() {
    //     return photonPoseEstimator.update().get().timestampSeconds;
    // }

    // public Pose2d getPhotonPose() {
    //     return photonPoseEstimator.update().get().estimatedPose.toPose2d();
    // }

    public double getVisionAngleEstimate() {
        return aprilTagBestTarget.getYaw();
    }

    public int getTagID() {
        return fiducialID;
    }
}