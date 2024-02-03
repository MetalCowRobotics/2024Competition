package frc.robot.subsystems;

import frc.robot.Constants;

import java.io.IOException;

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
    private PhotonTrackedTarget aprilTagBestTarget;
    private AprilTagFieldLayout aprilTagFieldLayout;
    private PhotonPoseEstimator photonPoseEstimator;
    private int fiducialID = -1;
    private Transform3d robotToCam;
    private double aprilTagAngle = 0;
    private Pose2d aprilTagPoseEstimate = new Pose2d();

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
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera ,robotToCam);
    }

    public boolean hasTargets() {
        return this.aprilTagHasTargets;
    }

    public Pose2d getaprilTagPoseEstimate() {
        return this.aprilTagPoseEstimate;
    }

    public double getTimestampSeconds() {
        return this.aprilTagResult.getTimestampSeconds();
    }

    public Rotation2d getAprilTagGyroYaw() {
        return Rotation2d.fromRadians(this.aprilTagAngle);
    }

    public int getID() {
        return this.fiducialID;
    }

    public void periodic() {
        aprilTagResult = camera.getLatestResult();
        aprilTagHasTargets = aprilTagResult.hasTargets();

        if (aprilTagHasTargets) {
            aprilTagBestTarget = aprilTagResult.getBestTarget();

            photonPoseEstimator.update();
            aprilTagPoseEstimate = photonPoseEstimator.update().get().estimatedPose.toPose2d();

            fiducialID = aprilTagBestTarget.getFiducialId();
            aprilTagAngle = aprilTagBestTarget.getBestCameraToTarget().getRotation().getAngle();
        }
    }
}