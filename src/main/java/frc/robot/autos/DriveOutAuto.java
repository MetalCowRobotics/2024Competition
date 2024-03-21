package frc.robot.autos;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.FullArmSubsystem;
import frc.lib14.MCRCommand;
import frc.lib14.SequentialCommands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib14.CommandPause;
import frc.lib14.ParallelCommands;
import frc.lib14.TimedCommandSet;

public class DriveOutAuto implements MCRCommand{    
    boolean first = true;
    double armMovementTimeout = .5;
    MCRCommand twoNoteAuto;
    
    public DriveOutAuto(Swerve s_Swerve, Intake m_Intake){
        twoNoteAuto = new SequentialCommands(
                new TimedCommandSet(new DriveToPointA(s_Swerve, m_Intake, -2.5, 0, s_Swerve.getGyroYaw().getDegrees()), 1.5)
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