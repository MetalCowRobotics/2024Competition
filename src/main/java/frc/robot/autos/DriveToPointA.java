package frc.robot.autos;

import frc.lib14.InstantCommandBase;
import frc.robot.subsystems.Swerve;

public class DriveToPointA extends InstantCommandBase{
    Swerve s_swerve;
    public DriveToPointA(Swerve m_swerve){
        s_swerve = m_swerve;
        }
    @Override
    public void run(){
        s_swerve.driveToPoint(1, 1, 0);
    }
}
