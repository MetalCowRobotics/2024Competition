package frc.robot.autos;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Swerve;
import frc.lib14.MCRCommand;
import frc.lib14.SequentialCommands;
import frc.lib14.CommandPause;

/*This auto just leaves the zone */
public class DriveOutAuto implements MCRCommand{    
    boolean first = true;
    double armMovementTimeout = .5;
    MCRCommand twoNoteAuto;
    
    public DriveOutAuto(Swerve s_Swerve, IntakeSubsystem m_Intake){
        twoNoteAuto = new SequentialCommands(
                new ZeroGyro(s_Swerve),
                // new DriveToPointA(s_Swerve, m_Intake,-2,0,0)
                new DriveToPointB(s_Swerve,-1.5,0,90),
                new CommandPause(1),
                new DriveToPointB(s_Swerve, -1.5, 2, 90)
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