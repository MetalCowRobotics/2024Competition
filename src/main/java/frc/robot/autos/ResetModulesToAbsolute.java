package frc.robot.autos;

import frc.lib14.InstantCommandBase;
import frc.robot.subsystems.Swerve;

public class ResetModulesToAbsolute extends InstantCommandBase{
    Swerve s_swerve;
    public ResetModulesToAbsolute(Swerve m_swerve){
        s_swerve = m_swerve;
    }
    @Override
    public void run(){
        s_swerve.resetModulesToAbsolute();
    }
}
