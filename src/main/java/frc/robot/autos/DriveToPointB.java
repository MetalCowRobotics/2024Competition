package frc.robot.autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib14.MCRCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class DriveToPointB implements MCRCommand {

    private Swerve m_swerve;

    private final double TOLERANCE = 0.2;
    private final double ANGLE_TOLERANCE = 0.75;

    private double targetX;
    private double targetY;
    private double targetAngle;

    private PIDController anglePIDController = new PIDController(0.00004, 0, 0.001);
    private PIDController xController = new PIDController(0.7, 0, 0.001);
    private PIDController yController = new PIDController(0.6, 0, 0);

    public DriveToPointB(Swerve swerve, double x, double y, double theta) {
        this.m_swerve = swerve;
        // addRequirements(m_swerve);

        anglePIDController.setSetpoint(0);
        anglePIDController.setTolerance(2);

        this.targetX = x;
        this.targetY = y;
        this.targetAngle = theta;

        xController.setSetpoint(targetX);
        yController.setSetpoint(targetY);
        anglePIDController.setSetpoint(targetAngle);
    }

    @Override
    public void run() {
        /* Get Values, Deadband*/

        double x = m_swerve.getPose().getX();
        double y = m_swerve.getPose().getY();

        SmartDashboard.putNumber("tracked x", x);
        SmartDashboard.putNumber("tracked y", y);
            
        xController.setSetpoint(targetX);
        yController.setSetpoint(targetY);

        // System.out.println("tx:" + targetX + ", ty:" + targetY);
        double yaw = m_swerve.getGyroYaw().getDegrees();

        yaw = yaw % 360;
        if (yaw < 0) {
            yaw += 360;
        }

        if (targetAngle == 0) {
            if (yaw > 180) {
                anglePIDController.setSetpoint(360);
            } else {
                anglePIDController.setSetpoint(0);
            }
        }

        double rotation = anglePIDController.calculate(yaw);
        double xCorrection = xController.calculate(x);
        double yCorrection = yController.calculate(y);
        // SmartDashboard.putNumber("absolute yaw", yaw);
        
        m_swerve.driveAuto(
            new Translation2d(xCorrection, yCorrection).times(Constants.Swerve.maxAutoSpeed), 
            -rotation * 0, 
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
        // if (yaw < 0) {
        //     yaw += 360;
        // }

        SmartDashboard.putBoolean("x tolerance", Math.abs(x - targetX) < TOLERANCE);
        SmartDashboard.putBoolean("y tolerance", Math.abs(y - targetY) < TOLERANCE);
        SmartDashboard.putBoolean("angle tolerance", Math.abs(yaw - targetAngle) < ANGLE_TOLERANCE);
        SmartDashboard.putNumber("angle error", Math.abs(yaw - targetAngle));

        if ( (Math.abs(x - targetX) < TOLERANCE && Math.abs(y - targetY) < TOLERANCE) && Math.abs(yaw - targetAngle) < ANGLE_TOLERANCE) {
            // m_swerve.setStop();
            return true;

        }
        return false;
    }
}