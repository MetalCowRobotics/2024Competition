package frc.robot.autos;

import frc.lib14.MCRCommand;
import frc.robot.subsystems.NoteTransitSubsystem;

public class ArmToAngles implements MCRCommand{

    String pos;
    boolean finished_flag = false;
    boolean first_time = true;

    public ArmToAngles(String position) {
        this.pos = position;
    }

    @Override
    public void run() {
        // if String pos is equal pickup
        if(pos.equals("pickup")){
            NoteTransitSubsystem.getInstance().setPickupPosition();
        }
        if(pos.equals("rest")){
            NoteTransitSubsystem.getInstance().setRestPosition();
        }
        if(pos.equals("speaker")){
            NoteTransitSubsystem.getInstance().setSpeakerPosition();
            
        }
        if(pos.equals("speakerFromNote")){
            NoteTransitSubsystem.getInstance().setSpeakerFromSpikeMark();
        }
        first_time = false;
    }

    @Override
    public boolean isFinished(){
        if(!first_time){
        if (!finished_flag && NoteTransitSubsystem.getInstance().atTarget()) 
            finished_flag = true;
        return finished_flag;
        }
        return false;
    }

}