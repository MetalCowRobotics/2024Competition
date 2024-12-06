package frc.robot.autos;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib14.InstantCommand;
import frc.robot.subsystems.Swerve;

/*This command resets the gyro to make it the starting position. */
public class ZeroGyro extends InstantCommand{
    Swerve m_swerve;
    public ZeroGyro(Swerve s_swerve){
        m_swerve = s_swerve;
        }
    @Override
    public void run(){
        // s_swerve.setHeading(new Rotation2d(180));

        m_swerve.zeroGyro();
        SmartDashboard.putString("auto", "zeroed the gyro");
    }
}
