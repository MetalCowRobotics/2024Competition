package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NoteTransitSubsystem;

/*This command sets the shooter and intake to different angles. */
public class ArmToAngles2 extends Command{
    String pos;
    boolean finished_flag = false;
    boolean first_time = true;
    public ArmToAngles2(String position){
        pos = position;
    }
    @Override
    public void execute() {
        
        if(pos.equals("pickupPosition")){
            NoteTransitSubsystem.getInstance().setPickupPosition();
        }
        if (pos.equals("stagePosition")) {
            NoteTransitSubsystem.getInstance().setStage();
        }
        if(pos.equals("restPosition")){
            NoteTransitSubsystem.getInstance().setRestPosition();
        }
        if(pos.equals("speakerPosition")){
            NoteTransitSubsystem.getInstance().setSpeakerPosition();
        }
        if(pos.equals("speakerFromNotePosition")){
            NoteTransitSubsystem.getInstance().setSpeakerFromSpikeMark();
        }
        if(pos.equals("speakerMidPosition")){
            NoteTransitSubsystem.getInstance().setSpeakerMidPosition();
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
