package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FullArmSubsystem;

public class ArmToAngles extends InstantCommand {
    
    FullArmSubsystem m_fullarm;
    String pos;

    public ArmToAngles(FullArmSubsystem m_fullarm, String pos) {
        this.m_fullarm = m_fullarm;
        this.pos = pos
    }

    @Override
    public void run() {
        // if String pos is equal pickup
        if(pos == "pickup"){
            m_fullarm.setPickupPosition();
        }
        if(pos == "rest"){
            m_fullarm.setRestPosition();
        }
        if(pos == "climb_start"){
            m_fullarm.setClimbVertPosition();
        }
        if(pos == "speaker"){
            m_fullarm.setSpeakerPosition();
        }
        if(pos == "climb_fin"){
            m_fullarm.setClimbFinPosition();
        }
        if(pos = "amp"){
            m_fullarm.setAmpPosition();
        }
    }

}