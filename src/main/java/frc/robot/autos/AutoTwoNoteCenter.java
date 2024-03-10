package frc.robot.autos;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.FullArmSubsystem;
import frc.lib14.MCRCommand;
import frc.lib14.SequentialCommands;
import frc.lib14.CommandPause;
import frc.lib14.ParallelCommands;
import frc.lib14.TimedCommandSet;

public class AutoTwoNoteCenter implements MCRCommand{    
    boolean first = true;
    double armMovementTimeout = .5;
    MCRCommand twoNoteAuto;
    
    public AutoTwoNoteCenter(Swerve s_Swerve, Intake m_Intake, Shooter m_Shooter, FullArmSubsystem m_FullArmSubsystem){
        // Shoot the notes into the speaker
        twoNoteAuto = new SequentialCommands(
            // start with resetting
            new ResetModulesToAbsolute(s_Swerve),

            // Setting the arm angles to poin the shooter and shoot the preloaded note
            new StartShooter(m_Shooter),
            new ArmToAngles(m_FullArmSubsystem, "speaker"),
            new ShooterReady(m_Shooter),
            new StartIntake(m_Intake),
            new CommandPause(.75),
            new StopShooter(m_Shooter),
            new StopIntake(m_Intake),

            // Set the arm angles to pick the note up that is in front of the robot
           
            // new StartIntake(m_Intake),
            
            new ArmToAngles(m_FullArmSubsystem, "pickup"),
            new StartIntake(m_Intake),
            new TimedCommandSet(new DriveToPointA(s_Swerve, m_Intake, -1.2, 0, 0), 7),
            new StopIntake(m_Intake),
            // new ArmToAngles(m_FullArmSubsystem, "rest")
            
        //    new StopIntake(m_Intake),
        //    new CommandPause(1),

           new ArmToAngles(m_FullArmSubsystem, "speaker"),

            // Setting the arm angles to speaker to shoot the picked up piece
            // new ArmToAngles(m_FullArmSubsystem, "speaker"),
            new StartShooter(m_Shooter),
            new ShooterReady(m_Shooter),
            new StartIntake(m_Intake),
            new CommandPause(.75),
            new StopShooter(m_Shooter),
            new StopIntake(m_Intake), 

            // Set the arm angles to stow
            new ArmToAngles(m_FullArmSubsystem, "rest")
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
