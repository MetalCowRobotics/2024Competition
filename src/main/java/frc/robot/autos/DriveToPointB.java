package frc.robot.autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib14.MCRCommand;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Swerve;

public class DriveToPointB implements MCRCommand {

    private Swerve m_swerve;

    private final double TOLERANCE = 0.2;

    private double targetX;
    private double targetY;
    private double targetAngle;

    private PIDController anglePIDController = new PIDController(0.02, 0, 0.00);
    private PIDController xController = new PIDController(0.7, 0.05, 0.00);
    private PIDController yController = new PIDController(0.8, 0, 0);

    public DriveToPointB(Swerve swerve, double x, double y, double theta) {
        this.m_swerve = swerve;

        anglePIDController.setSetpoint(0);
        anglePIDController.setTolerance(3);
        anglePIDController.enableContinuousInput(0, 360); // restarts from 0 instead of going over 360 for PID calculations
        xController.setTolerance(TOLERANCE);
        yController.setTolerance(TOLERANCE);

        // Setting target x, y, and target angle
        this.targetX = x;
        this.targetY = y;
        this.targetAngle = theta;

        // Setting these targets to the pid controllers.
        xController.setSetpoint(targetX);
        yController.setSetpoint(targetY);
        anglePIDController.setSetpoint(targetAngle);
    }

    // Corrects the angle to be within the range of 0 to 360.
    public double correctAngle(double yaw)
    {
        yaw = yaw % 360;
        if (yaw < 0) {
            yaw += 360;
        }
        return yaw;
    }

    @Override
    public void run() {
        /* Get Values, Deadband*/
        SmartDashboard.putBoolean("stoppedRobot", false);

        // get the x and y position of the robot with respect to the field.
        double x = m_swerve.getPose().getX();
        double y = m_swerve.getPose().getY();

        SmartDashboard.putNumber("tracked x", x);
        SmartDashboard.putNumber("tracked y", y);
        
        // setting the PIDController to the target position of the robot.
        xController.setSetpoint(targetX);
        yController.setSetpoint(targetY);

        // Get the current angle of rotation of the robot's Gyro.
        double yaw = m_swerve.getGyroYaw().getDegrees();
        yaw = correctAngle(yaw);
        
        // Find the shortest way to reach the zero angle rotation.
        if (targetAngle == 0) {
            if (yaw > 180) {
                anglePIDController.setSetpoint(360);
            } else {
                anglePIDController.setSetpoint(0);
            }
        }

        // calculate the correction for the PID loop
        double rotation = anglePIDController.calculate(yaw);
        double xCorrection = xController.calculate(x);
        double yCorrection = yController.calculate(y);

        // Math.abs(x - targetX) < TOLERANCE && Math.abs(y - targetY) < TOLERANCE
        // If the robot is at the target angle and position
        if (xController.atSetpoint() && yController.atSetpoint()){
            xCorrection = 0;
            yCorrection = 0;
        }
        if(anglePIDController.atSetpoint()){
            rotation = 0;
        }

        // drive the robot based on the correction.
        m_swerve.driveAuto(
            new Translation2d(xCorrection, yCorrection).times(Constants.Swerve.maxAutoSpeed), 
            rotation, 
            true, 
            false
        );
    }

    @Override
    public boolean isFinished() {
        double x = m_swerve.getPose().getX();
        double y = m_swerve.getPose().getY();
        double yaw = m_swerve.getGyroYaw().getDegrees();

        yaw = yaw % 360;
        yaw = correctAngle(yaw);

        System.out.println("Angle: " + yaw);
        SmartDashboard.putNumber("yaw", yaw);
        System.out.println(("x tolerance: " + (Math.abs(x - targetX) < TOLERANCE)));
        System.out.println(("y tolerance: " +  (Math.abs(y - targetY) < TOLERANCE)));
        SmartDashboard.putBoolean("angle tolerance", Math.abs(yaw - targetAngle) < 3);
        SmartDashboard.putNumber("angle error", Math.abs(yaw - targetAngle));
        System.out.println("ifStatement: " + (Math.abs(x - targetX) < TOLERANCE && Math.abs(y - targetY) < TOLERANCE));

        // Stop the robot and return true if the robot is at the target.
        if ((xController.atSetpoint() && yController.atSetpoint() && anglePIDController.atSetpoint())||IntakeSubsystem.getInstance().noteAcquired()){
            m_swerve.stop();
            return true;
        }
        return false;
    }
}