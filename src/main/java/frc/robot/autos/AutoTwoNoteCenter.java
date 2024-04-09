package frc.robot.autos;

import frc.robot.subsystems.Swerve;
import frc.lib14.MCRCommand;
import frc.lib14.SequentialCommands;
import frc.lib14.TimedCommandSet;
import frc.lib14.CommandPause;
import frc.lib14.ParallelCommands;

/*This auto shoots the preloaded note and drives to pick the note on the center spike mark and shoots that into the speaker.
 * points: 12
 */
public class AutoTwoNoteCenter implements MCRCommand{    
    boolean first = true;
    double armMovementTimeout = .5;
    MCRCommand twoNoteAuto;
    
    public AutoTwoNoteCenter(Swerve s_Swerve){
        twoNoteAuto = new SequentialCommands(
            // start with resetting
            //new ZeroGyro(s_Swerve),

            //new ArmToAngles("rest"),            
            // Setting the arm angles to point the shooter and shoot the preloaded note
            new ToggleShooter(),
            new TimedCommandSet(new ArmToAngles("speaker"), 1.5),
            

            new EjectIntake(),
            //new CommandPause(1),
            new CommandPause(2)
        //    // new ToggleShooter(),
            // new StopIntake(),

        //     // Set the arm angles to pick the note up that is in front of the robot
        //      new ArmToAngles("pickup"),
        //     new StartIntake(),
        //     new TimedCommandSet(new DriveToPointB(s_Swerve, 1.4, 0, 0), 1.8),
        //     new CommandPause(.5),
            
        // // //     // Setting the arm angles to speaker to shoot the picked up piece
        //    // new ParallelCommands(
        //         new ArmToAngles("speaker"),
        //         new DriveToPointB(s_Swerve, 0, 0, 0),
        //    // ),
        //     new ToggleShooter(),
        //     new ToggleShooter(),
        //     new CommandPause(1),
        //     new EjectIntake(),
        //     new CommandPause(1.5),
        //     new ToggleShooter(),
        //     new StopIntake()

        // // //     // Set the arm angles to stow
        //     // new ArmToAngles("rest")

        
         );
    }
    
    @Override
    public void run(){
        twoNoteAuto.run();
    }
    
    @Override
    public boolean isFinished(){
        return twoNoteAuto.isFinished();
    }
}