package frc.robot.autos;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.NoteTransitSubsystem;
import frc.lib14.MCRCommand;
import frc.lib14.SequentialCommands;
import frc.lib14.CommandPause;
import frc.lib14.ParallelCommands;
import frc.lib14.TimedCommandSet;

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
            new ZeroGyro(s_Swerve),

            new ArmToAngles("rest"),            
            // Setting the arm angles to point the shooter and shoot the preloaded note
            new ToggleShooter(),
            new ArmToAngles("speaker"),
            new CommandPause(1),
            new StartIntake(),
            new CommandPause(1.5),
            new ToggleShooter(),
            new StopIntake(),

            // Set the arm angles to pick the note up that is in front of the robot
            new StartIntake(),
            new ArmToAngles("pickup"),
            new DriveToPointB(s_Swerve, -1.5, 0, 0),

            // Setting the arm angles to speaker to shoot the picked up piece
            new ToggleShooter(),
            new ArmToAngles("speakerFromNote"),
            new CommandPause(1),
            new StartIntake(),
            new CommandPause(1.5),
            new ToggleShooter(),
            new StopIntake(),

            // Set the arm angles to stow
            new ArmToAngles("rest")
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