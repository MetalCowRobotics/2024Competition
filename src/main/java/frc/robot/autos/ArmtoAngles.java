package frc.robot.autos;

import frc.robot.subsystems.FullArmSubsystem;
import frc.lib14.MCRCommand;

public class ArmtoAngles implements MCRCommand{
    
    FullArmSubsystem m_fullarm;
    String pos;
    boolean finished_flag = false;

    public ArmtoAngles(FullArmSubsystem m_fullarmSub, String position) {
        this.m_fullarm = m_fullarmSub;
        this.pos = position;
    }

    @Override
    public void run() {
        // if String pos is equal pickup
        if(pos.equals("pickup")){
            m_fullarm.setPickupPosition();
        }
        if(pos.equals("rest")){
            m_fullarm.setRestPosition();
        }
        if(pos.equals("climb_start")){
            m_fullarm.setClimbVertPosition();
        }
        if(pos.equals("speaker")){
            m_fullarm.setSpeakerPosition();
        }
        if(pos.equals("climb_fin")){
            m_fullarm.setClimbFinPosition();
        }
    }

    @Override
    public boolean isFinished(){
        if (!finished_flag && m_fullarm.atTarget()) 
            finished_flag = true;
        return finished_flag;
    }

}