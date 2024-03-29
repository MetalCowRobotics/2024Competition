package frc.robot.autos;

import frc.robot.subsystems.Swerve;
import frc.lib14.CommandPause;
import frc.lib14.MCRCommand;
import frc.lib14.SequentialCommands;

/*This auto shoots the preloaded note into the speaker and leave the zone after 10 seconds */
public class AutoShootAndLeave {
    MCRCommand twoNoteAuto;

    public AutoShootAndLeave(Swerve s_Swerve){
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
            new DriveToPointB(s_Swerve, -2.0, 0, 60.0)
           
        );
    

    }


}
