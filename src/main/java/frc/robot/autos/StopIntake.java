package frc.robot.autos;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib14.InstantCommandBase;
import frc.robot.subsystems.NoteTransitSubsystem;

/*This command disables the intake */
public class StopIntake extends InstantCommandBase{
    public StopIntake(){
        }
    @Override
    public void run(){
        NoteTransitSubsystem.getInstance().disableIntake();
        SmartDashboard.putString("auto", "stopped");
    }
}
