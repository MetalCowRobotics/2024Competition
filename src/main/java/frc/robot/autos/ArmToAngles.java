package frc.robot.autos;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib14.MCRCommand;
import frc.robot.subsystems.NoteTransitSubsystem;

/*This command sets the shooter and intake to different angles. */
public class ArmToAngles implements MCRCommand{
    String pos;
    boolean finished_flag = false;
    boolean first_time = true;

    public ArmToAngles(String position) {
        this.pos = position;
    }

    @Override
    public void run() {
        // SmartDashboard.putString("ArmToAngles", "started running");
        if(pos.equals("pickup")){
            SmartDashboard.putString("ArmToAngles", "pickup");
            NoteTransitSubsystem.getInstance().setPickupPosition();
        }
        if(pos.equals("rest")){
            SmartDashboard.putString("ArmToAngles", "rest");
            NoteTransitSubsystem.getInstance().setRestPosition();
        }
        if(pos.equals("speaker")){
            SmartDashboard.putString("ArmToAngles", "speaker");
            NoteTransitSubsystem.getInstance().setSpeakerPosition();
        }
        if(pos.equals("speakerFromNote")){
            SmartDashboard.putString("ArmToAngles", "speaker");
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
