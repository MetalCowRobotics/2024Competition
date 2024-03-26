package frc.robot.autos;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib14.InstantCommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.NoteTransitSubsystem;

/*This command enables the intake */
public class StartIntake extends InstantCommandBase{
    @Override
    public void run(){
        NoteTransitSubsystem.getInstance().enableIntake();
        // i_intake.setIntakeTrue();
        SmartDashboard.putString("auto", "running");
    }
}
