package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

public class Swerve {
    public SwerveDrivePoseEstimator swervePoseEstimator;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    private Vision m_vision;
    private int lastTargetID;
    private boolean visionControl;

    private final double accelerationTime = 0.3;
    private double speedMultiplier = 1;
    private double desiredSpeed = Constants.Swerve.maxSpeed * speedMultiplier;

    private double linearAcceleration = desiredSpeed / accelerationTime;

    private SlewRateLimiter m_xSlewRateLimiter = new SlewRateLimiter(linearAcceleration, -linearAcceleration, 0);
    private SlewRateLimiter m_ySlewRateLimiter = new SlewRateLimiter(linearAcceleration, -linearAcceleration, 0);

    private PIDController angleHoldingPIDController = new PIDController(0.0004, 0, 0);
    private PIDController xController = new PIDController(0.6, 0, 0);
    private PIDController yController = new PIDController(0.6, 0, 0);
    private PIDController thetaController = new PIDController(0.004, 0, 0);

    private boolean positionReached = false;

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);
        xController.setTolerance(0.05);
        thetaController.setTolerance(2);
        thetaController.enableContinuousInput(0, 360);

        m_vision = new Vision();
        lastTargetID = -1;
        visionControl = false;
        
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        swervePoseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions(), new Pose2d());
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        double xSpeed = m_xSlewRateLimiter.calculate(translation.getX() * speedMultiplier);
        double ySpeed = m_ySlewRateLimiter.calculate(translation.getY() * speedMultiplier);
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    xSpeed, 
                                    ySpeed, 
                                    rotation, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    xSpeed, 
                                    ySpeed, 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, desiredSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
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

    public void setPose(Pose2d pose) {
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
        m_vision.periodic();
        swervePoseEstimator.resetPosition(new Rotation2d(m_vision.getVisionAngleEstimate()), getModulePositions(), m_vision.getVisionPoseEstimate());
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public void setSprint() {
        speedMultiplier = 0.833;
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

        drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            false /* KEEP FALSE */
        );
    }
        
    public void driveToPoint(double targetX, double targetY, double targetTheta) {
        double x = getPose().getX();
        double y = getPose().getY();
        double yaw = getGyroYaw().getDegrees();
        positionReached = false;

        xController.setSetpoint(targetX);
        yController.setSetpoint(targetY);

        yaw = yaw % 360;
        if (yaw < 0) {
            yaw += 360;
        }
        
            if (yaw > 180) {
                thetaController.setSetpoint(360);
            } else {
                thetaController.setSetpoint(0);
            }

        double xCorrection = xController.calculate(x);
        double yCorrection = yController.calculate(y);
        //double rotation = thetaController.calculate(yaw);

        // Check if the robot is close enough to the target position
    if (Math.abs(x - targetX) < Constants.targetPositionTolerance &&
        Math.abs(y - targetY) < Constants.targetPositionTolerance &&
        Math.abs(yaw - targetTheta) < Constants.targetAngleTolerance) {
        // Stop the robot by setting the desired states to zero
        setLocationReached();
        SwerveModuleState[] stopStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            stopStates[i] = new SwerveModuleState(0, new Rotation2d());
        }
        setModuleStates(stopStates);
        } else {
            driveAuto(
                new Translation2d(xCorrection, yCorrection).times(Constants.Swerve.maxAutoSpeed), 
                -yaw * 0, 
                true, 
                false
            );
        }
    }

    public void visionAndPosePeriodic() {
        swervePoseEstimator.updateWithTime(Timer.getFPGATimestamp(), getGyroYaw(), getModulePositions());

        m_vision.periodic();
        if (m_vision.hasTargets()) {
            if (lastTargetID == -1) {
                setPose(m_vision.getVisionPoseEstimate());
            }
            swervePoseEstimator.addVisionMeasurement(m_vision.getVisionPoseEstimate(), m_vision.getTimestampSeconds());
            lastTargetID = m_vision.getTagID();
        }
        else {
            lastTargetID = -1;
        }
        SmartDashboard.putNumber("Vision X Pose", m_vision.getVisionPoseEstimate().getX());
        SmartDashboard.putNumber("Vision Y Pose", m_vision.getVisionPoseEstimate().getY());
        SmartDashboard.putNumber("Vision Pose Angle", m_vision.getVisionAngleEstimate());

        SmartDashboard.putNumber("AprilTagID", lastTargetID);
        SmartDashboard.putNumber("X Pose", getPose().getX());
        SmartDashboard.putNumber("Y Pose", getPose().getY());
        SmartDashboard.putNumber("Pose Angle", getHeading().getDegrees());
    }

    public void periodic(DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        visionAndPosePeriodic();

        if (visionControl) {
            // [NEW DRIVETOPOINT METHOD]
        } else {
            teleopSwerve(translationSup, strafeSup, rotationSup, robotCentricSup);
        }
    }
}