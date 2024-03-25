package frc.robot.autos;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib14.InstantCommandBase;
import frc.robot.subsystems.Shooter;

public class StartShooter extends InstantCommandBase{
    Shooter s_shooter;
    public StartShooter(Shooter m_shooter){
        s_shooter = m_shooter;
        }
    @Override
    public void run(){
        // s_swerve.setHeading(new Rotation2d(180));
        s_shooter.setShootingSpeed();
        SmartDashboard.putString("auto", "running");
    }
}
