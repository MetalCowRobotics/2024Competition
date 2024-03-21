package frc.robot.autos;

import frc.lib14.MCRCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Shooter;

public class ShooterReady implements MCRCommand{
    Shooter s_Shooter;
    boolean finished_flag = false;

    public ShooterReady(Shooter m_Shooter){
        s_Shooter = m_Shooter;

    }
    @Override
    public void run(){
        // i_Intake.startIntake();
    
    }

    @Override
    public boolean isFinished(){
        if(!finished_flag && s_Shooter.getShooterSpunUp()){
            finished_flag = true;
        }
        return finished_flag;
    }
}
