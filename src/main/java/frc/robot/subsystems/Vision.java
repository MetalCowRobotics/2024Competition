package frc.robot.subsystems;

import frc.robot.Constants;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

    private String cameraName;
    private PhotonCamera camera;
    private PhotonPipelineResult aprilTagResult;
    private boolean aprilTagHasTargets;
    private PhotonTrackedTarget aprilTagBestTarget;
    private AprilTagFieldLayout aprilTagFieldLayout;
    private PhotonPoseEstimator photonPoseEstimator;
    private int fiducialID;
    private Transform3d robotToCam;

    public Vision() {
        camera = new PhotonCamera("MicrosoftLifeCamHD-3000");
        aprilTagResult = new PhotonPipelineResult();
        aprilTagHasTargets = false;
        aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        robotToCam = Constants.VisionConstants.robotToCamTranslation;
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
    }

    public void periodic(){
        photonPoseEstimator.update();

        aprilTagResult = camera.getLatestResult();
        aprilTagHasTargets = aprilTagResult.hasTargets();

        if (aprilTagHasTargets) {
            aprilTagBestTarget = aprilTagResult.getBestTarget();
            fiducialID = aprilTagBestTarget.getFiducialId();
        } 
        else {
            fiducialID = -1;
        }

        SmartDashboard.putNumber(cameraName + " Fiducial ID", fiducialID);
    }

    public double getTimestampSeconds(){
        return this.aprilTagResult.getTimestampSeconds();
    }

    public boolean hasTargets(){
        return this.aprilTagHasTargets;
    }

    public Pose2d getVisionPoseEstimate() {
        return photonPoseEstimator.getReferencePose().toPose2d();
    }

    public double getVisionAngleEstimate() {
        return aprilTagBestTarget.getYaw();
    }

    public int getTagID(){
        return fiducialID;
    }
}