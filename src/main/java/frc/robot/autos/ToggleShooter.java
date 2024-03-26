package frc.robot.autos;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib14.InstantCommandBase;
import frc.robot.subsystems.NoteTransitSubsystem;

public class ToggleShooter extends InstantCommandBase{
    @Override
    public void run(){
        // s_swerve.setHeading(new Rotation2d(180));
        NoteTransitSubsystem.getInstance().toggleShooter();
        SmartDashboard.putString("auto", "running");
    }
}
