package frc.robot.autos;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib14.InstantCommandBase;
import frc.robot.subsystems.NoteTransitSubsystem;

/*This command quickly ejects the note out of the intake */
public class EjectIntake extends InstantCommandBase{
    
    public EjectIntake(){
        }
    @Override
    public void run(){
        // s_swerve.setHeading(new Rotation2d(180));

        NoteTransitSubsystem.getInstance().quickOuttake();
        // i_intake.setIntakeTrue();
        SmartDashboard.putString("auto", "running");
    }
}
