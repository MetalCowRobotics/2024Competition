package frc.robot.autos;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib14.InstantCommand;
import frc.robot.subsystems.NoteTransitSubsystem;

/*This command disables the intake */
public class StopIntake extends InstantCommand{
    public StopIntake(){
        }
    @Override
    public void run(){
        NoteTransitSubsystem.getInstance().disableIntake();
        SmartDashboard.putString("auto", "stopped");
    }
}
