package frc.robot.autos;

import frc.lib14.InstantCommandBase;
import frc.robot.subsystems.Swerve;

public class DriveToPointA extends InstantCommandBase{
    Swerve s_swerve;
    double xCor;
    double yCor;
    double targetAngle;
    public DriveToPointA(Swerve m_swerve, double x, double y, double theta){
        s_swerve = m_swerve;
        xCor = x;
        yCor = y;
        targetAngle = theta;
        }
    @Override
    public void run(){
        s_swerve.driveToPoint(xCor, yCor, targetAngle);
    }
}
