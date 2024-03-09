package frc.robot.autos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib14.InstantCommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class DriveToPointA extends InstantCommandBase{
    Swerve s_swerve;
    double xCor;
    double yCor;
    double targetAngle;
    double XY_TOLERANCE = 0.2;
    double ANGLE_TOLERANCE = 0.75;

    public DriveToPointA(Swerve m_swerve, double x, double y, double theta){
        s_swerve = m_swerve;
        xCor = x;
        yCor = y;
        targetAngle = theta;
        }

    @Override
    public void run(){
        double x = s_swerve.getPose().getX();
        double y = s_swerve.getPose().getY();
        double yaw = s_swerve.getGyroYaw().getDegrees();

        yaw = yaw % 360;

        if ( (Math.abs(x - xCor) < XY_TOLERANCE && Math.abs(y - yCor) < XY_TOLERANCE) && Math.abs(yaw - targetAngle) < ANGLE_TOLERANCE) {
            s_swerve.drive(
                new Translation2d(0, 0).times(Constants.Swerve.maxSpeed),
                s_swerve.getGyroYaw().getDegrees(),
                true,
                false
            );
        }
        else {
            s_swerve.driveToPoint(xCor, yCor, targetAngle);
        }
    }
}
