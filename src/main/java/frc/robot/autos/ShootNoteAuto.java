package frc.robot.autos;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.OldCode.FullArmSubsystem;
import frc.lib14.MCRCommand;
import frc.lib14.SequentialCommands;
import frc.lib14.CommandPause;
import frc.lib14.ParallelCommands;
import frc.lib14.TimedCommandSet;

public class ShootNoteAuto implements MCRCommand{    
    boolean first = true;
    double armMovementTimeout = .5;
    MCRCommand twoNoteAuto;
    
    public ShootNoteAuto(Swerve s_Swerve, IntakeSubsystem m_Intake, Shooter m_Shooter, FullArmSubsystem m_FullArmSubsystem){
        // Shoot the notes into the speaker
        twoNoteAuto = new SequentialCommands(
            
            // Setting the arm angles to poin the shooter and shoot the preloaded note
            new ToggleShooter(m_Shooter),
            new ArmToAngles(m_FullArmSubsystem, "speaker"),
            new TimedCommandSet(new ShooterReady(m_Shooter), 1.5),
            new FeedNote(m_Intake),
            new CommandPause(.75),
            new StopShooter(m_Shooter),
            new StopIntake(m_Intake)
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
