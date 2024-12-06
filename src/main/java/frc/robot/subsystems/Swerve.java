package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

public class Swerve implements Subsystem{
    public SwerveDrivePoseEstimator swervePoseEstimator;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    private Vision m_vision;
    private int lastTargetID;
    private boolean visionControl;
    private EstimatedRobotPose visionPose;
    private boolean visionToggle;

    private final double accelerationTime = 0.3;
    private double speedMultiplier = 1;
    private double desiredSpeed = Constants.Swerve.maxSpeed * speedMultiplier;

    private double linearAcceleration = desiredSpeed / accelerationTime;

    private SlewRateLimiter m_xSlewRateLimiter = new SlewRateLimiter(linearAcceleration, -linearAcceleration, 0);
    private SlewRateLimiter m_ySlewRateLimiter = new SlewRateLimiter(linearAcceleration, -linearAcceleration, 0);

    private PIDController angleHoldingPIDController = new PIDController(0.0004, 0, 0.001);
    private PIDController xController = new PIDController(0.6, 0, 0);
    private PIDController yController = new PIDController(0.6, 0, 0);
    private PIDController thetaController = new PIDController(0.035, 0, 0);
    private PIDController thetaController2 = new PIDController(0.0002, 0, 0);

    private boolean positionReached = false;

    public Swerve() {
        
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);
        xController.setTolerance(0.2);
        yController.setTolerance(0.2);
        thetaController.setTolerance(3);
        thetaController2.setTolerance(3);
        thetaController.enableContinuousInput(0, 360);
        thetaController2.enableContinuousInput(0, 360);
        visionToggle = true;

        m_vision = new Vision();
        lastTargetID = -1;
        visionControl = false;
        visionPose = new EstimatedRobotPose(new Pose3d(), 0, null, null);
        
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        swervePoseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions(), new Pose2d());
    }

public void setDriveOffsets(){
    mSwerveMods[0].setAngleOffset();
    mSwerveMods[1].setAngleOffset();
    mSwerveMods[2].setAngleOffset();
    mSwerveMods[3].setAngleOffset();


}

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        double xSpeed = m_xSlewRateLimiter.calculate(translation.getX() * speedMultiplier);
        SmartDashboard.putNumber("xTarget", translation.getX());
        double ySpeed = m_ySlewRateLimiter.calculate(translation.getY() * speedMultiplier);
        SmartDashboard.putNumber("yTarget", translation.getX());
        //SmartDashboard.putNumber("RotationTarget", rotation);
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    xSpeed, 
                                    ySpeed, 
                                    -rotation, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    xSpeed, 
                                    ySpeed, 
                                    -rotation
                                )
            );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, desiredSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public double getTotalDist(){
        if((m_vision.getTotalDist()-0.7112) > 0.0){
            return (m_vision.getTotalDist()-0.7112);
        }
        return 0.0;
    }

    public void driveAuto(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        double xSpeed = translation.getX();
        double ySpeed = translation.getY();
        double angularVelocity = rotation;
        angleHoldingPIDController.setSetpoint(getGyroYaw().getDegrees());
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    xSpeed, 
                                    ySpeed, 
                                    angularVelocity, 
                                    getGyroYaw()
                                )
                                : new ChassisSpeeds(
                                    xSpeed, 
                                    ySpeed, 
                                    angularVelocity
                                )
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed * speedMultiplier);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    // Stops the robot from moving
    public void stop(){
        SmartDashboard.putBoolean("stoppedRobot", true);
        driveAuto(new Translation2d(0,0), 0, true, false);
    }
           
    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, desiredSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return swervePoseEstimator.getEstimatedPosition();
    }

    // public Transform3d distFromTag(){
    //     return m_vision.getDistFromScoringTag();
    // }

    public ChassisSpeeds getRobotRelativeSpeeds(){
        return new ChassisSpeeds();
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        Translation2d translation = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        double rotation = speeds.omegaRadiansPerSecond;
        drive(translation, rotation, false, false);
        SmartDashboard.putString("bob", "3");
    }
    // public double getAngleFromTag(){
    //     return Math.atan2(distFromTag().getY(), distFromTag().getX());
    // }
    // public double getAngleFromTag2(){
    //     return m_vision.getYawOfBestTarget();
    // }

    public void resetPose(Pose2d pose) {
        swervePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading){
        swervePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroGyro() {
        swervePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public void visionToGyro() {
        m_vision.getPoseEstimate().ifPresent(estimatedRobotPose -> swervePoseEstimator.resetPosition(
            estimatedRobotPose.estimatedPose.toPose2d().getRotation(),
            getModulePositions(),
            estimatedRobotPose.estimatedPose.toPose2d()
        ));
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public void setSprint() {
        speedMultiplier = 1.3;
    }

    public void setCrawl() {
        speedMultiplier = 0.4375;
    }

    public void setBase() {
        speedMultiplier = 1;
    }

    public void enableVisionControl() {
        visionControl = true;
    }
    public void setStop() {
        speedMultiplier = 0;
    }

    public void setLocationReached(){
        positionReached = true;
    }

    public boolean isLocationReached(){
        return positionReached;
    }

    public void disableVisionControl() {
        visionControl = false;
    }

    public void periodicValues(){
        swervePoseEstimator.update(getGyroYaw(), getModulePositions());
        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
    }

    public void teleopSwerve(DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);
        SmartDashboard.putNumber("RotationTarget",rotationVal);

        drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            false /* KEEP FALSE */
        );
    }

    public void visionDriveToPoint(double targetX, double targetY, double targetTheta) {
        double x = getPose().getX();
        double y = getPose().getY();
        double yaw = getPose().getRotation().getDegrees();

        xController.setSetpoint(targetX);
        yController.setSetpoint(targetY);

        if (targetTheta == 0) {
            if (yaw > 180) {
                thetaController.setSetpoint(360);
            } else {
                thetaController.setSetpoint(0);
            }
        } else {
            thetaController.setSetpoint(targetTheta);
        }

        double rotation = thetaController.calculate(yaw);
        double xCorrection = xController.calculate(x);
        double yCorrection = yController.calculate(y);

        if (xController.atSetpoint() && yController.atSetpoint()){
            xCorrection = 0;
            yCorrection = 0;
        }
        if(thetaController.atSetpoint()){
            rotation = 0;
        }

        driveAuto(
            new Translation2d(xCorrection, yCorrection).times(Constants.Swerve.maxAutoSpeed), 
            -rotation, 
            true, 
            false
        );
    }

    public void visionAlign() {
        if (visionToggle) {
            setHeading(new Rotation2d(Math.toDegrees(m_vision.getYawOfBestTarget())));
        }
        visionToggle = false;
        thetaController.setSetpoint(0);
        double rotation = thetaController.calculate(getPose().getRotation().getDegrees());

        if(thetaController.atSetpoint()){
            rotation = 0;
        }

        driveAuto(
            new Translation2d(0, 0).times(Constants.Swerve.maxAutoSpeed), 
            -rotation, 
            true, 
            false
        );
    }

    public void visionAndPosePeriodic() {
        m_vision.getPoseEstimate().ifPresent(estimatedRobotPose -> visionPose = estimatedRobotPose);

        SmartDashboard.putNumber("Vision X Pose", visionPose.estimatedPose.getX());
        SmartDashboard.putNumber("Vision Y Pose", visionPose.estimatedPose.getY());

        swervePoseEstimator.addVisionMeasurement(visionPose.estimatedPose.toPose2d(), visionPose.timestampSeconds);

        SmartDashboard.putNumber("Vision Angle", visionPose.estimatedPose.toPose2d().getRotation().getDegrees());
        SmartDashboard.putNumber("Best Target Angle", m_vision.getYawOfBestTarget());
    }

    public void periodic(DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        visionAndPosePeriodic();
        if (visionControl) {
            // visionDriveToPoint(getPose().getX(), getPose().getY(), 178);
            visionAlign();
        }
        else {
            teleopSwerve(translationSup, strafeSup, rotationSup, robotCentricSup);
            visionToggle = true;
        }
        SmartDashboard.putNumber("X Pose", getPose().getX());
        SmartDashboard.putNumber("Y Pose", getPose().getY());
        SmartDashboard.putNumber("Drive Angle", getPose().getRotation().getDegrees());
        SmartDashboard.putBoolean("Vision Toggle", visionToggle);
    }
}