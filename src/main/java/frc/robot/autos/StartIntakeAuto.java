package frc.robot.autos;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib14.InstantCommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.NoteTransitSubsystem;

public class StartIntakeAuto extends InstantCommandBase{
    
   @Override
    public void run(){
        SmartDashboard.putString("auto", "enables intake");
        // IntakeSubsystem.getInstance().setAlreadyStopped(true);
        NoteTransitSubsystem.getInstance().enableIntake();
        // i_intake.setIntakeTrue();
        
    }

    @Override
    public boolean isFinished(){
        if(IntakeSubsystem.getInstance().noteAcquired()){
            NoteTransitSubsystem.getInstance().disableIntake();
            return true;
        }
        return false;
    }
}
