package frc.robot.autos;

import frc.lib14.MCRCommand;
import frc.robot.subsystems.NoteTransitSubsystem;

public class ShooterReady implements MCRCommand{
    boolean finished_flag = false;

    public ShooterReady(){
    }
    @Override
    public void run(){
    }

    @Override
    public boolean isFinished(){
        if(!finished_flag && NoteTransitSubsystem.getInstance().getShooterSpunUp()){
            finished_flag = true;
        }
        return finished_flag;
    }
}
