package frc.robot.autos;

import frc.lib14.InstantCommand;
import frc.robot.subsystems.Swerve;

public class ResetModulesToAbsolute extends InstantCommand{
    Swerve s_swerve;
    public ResetModulesToAbsolute(Swerve m_swerve){
        s_swerve = m_swerve;
    }
    @Override
    public void run(){
        s_swerve.zeroGyro();
    }
}
