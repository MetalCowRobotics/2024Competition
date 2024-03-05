package frc.robot.autos;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib14.InstantCommandBase;
import frc.robot.subsystems.Swerve;

public class Turn extends InstantCommandBase{
    Swerve s_swerve;
    double heading;
    public Turn(Swerve m_swerve, double angle){
        s_swerve = m_swerve;
        heading = angle;
        }
    @Override
    public void run(){
        s_swerve.setHeading(new Rotation2d(heading));
    }
}

