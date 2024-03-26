package frc.robot.autos;

import frc.robot.subsystems.NoteTransitSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.lib14.CommandPause;
import frc.lib14.MCRCommand;
import frc.lib14.SequentialCommands;
import frc.lib14.TimedCommandSet;
import frc.robot.autos.*;

public class AutoShootAndLeave {
    MCRCommand twoNoteAuto;

    public AutoShootAndLeave(Swerve s_Swerve, Intake m_Intake, Shooter m_Shooter, FullArmSubsystem m_FullArmSubsystem){
        twoNoteAuto = new SequentialCommands(
            // sets the position for shooting into speaker
            new ArmToAngles("speaker"),
            
            // shoots the note in the speaker
            new ToggleShooter(),
            
            // after shooting the shooter goes to rest position
            new ArmToAngles("rest"),
            
            //stops the robot for 10 seconds
            new CommandPause(10),
            
            // after 10 seconds it goes out of the zone
            new DriveToPointB(s_Swerve, m_Intake, -2.0, 0, 60.0)
           
        );
    

    }


}
