package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.NoteTransitSubsystem;

/*This command toggles the shooter on/off */
public class RunShooter extends Command{
    @Override
    public void execute(){
        NoteTransitSubsystem.getInstance().toggleShooter();
        SmartDashboard.putString("auto", "running");
    }
}
