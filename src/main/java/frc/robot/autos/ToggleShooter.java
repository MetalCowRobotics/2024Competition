package frc.robot.autos;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib14.InstantCommandBase;
import frc.robot.subsystems.NoteTransitSubsystem;

/*This command toggles the shooter on/off */
public class ToggleShooter extends InstantCommandBase{
    @Override
    public void run(){
        NoteTransitSubsystem.getInstance().toggleShooter();
        SmartDashboard.putString("auto", "running");
    }
}
