package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NoteTransitSubsystem;
import frc.robot.subsystems.Swerve;

/*This command sets the shooter and intake to different angles. */
public class VariablePos extends Command{
    boolean finished_flag = false;
    boolean first_time = true;
    public VariablePos(){
    }
    @Override
    public void execute() {
        NoteTransitSubsystem.getInstance().setVariableAngle(Swerve.getInstance().getTotalDist());
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
