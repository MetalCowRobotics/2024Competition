package frc.robot.autos;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib14.InstantCommand;
import frc.robot.subsystems.NoteTransitSubsystem;

/*This command toggles the shooter on/off */
public class ToggleShooter extends InstantCommand{
    @Override
    public void run(){
        NoteTransitSubsystem.getInstance().toggleShooter();
        SmartDashboard.putString("auto", "shooter running");
    }
}
