package frc.robot.autos;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib14.InstantCommandBase;
import frc.robot.subsystems.Swerve;

public class ZeroGyro extends InstantCommandBase{
    Swerve m_swerve;
    public ZeroGyro(Swerve s_swerve){
        m_swerve = s_swerve;
        }
    @Override
    public void run(){
        // s_swerve.setHeading(new Rotation2d(180));

        m_swerve.zeroGyro();
        SmartDashboard.putString("auto", "running");
    }
}
