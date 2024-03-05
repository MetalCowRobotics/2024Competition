package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.FullArmSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.LED;
import frc.lib14.MCRCommand;
import frc.lib14.SequentialCommands;
import frc.lib14.ParallelCommands;
import frc.lib14.InstantCommandBase;


class Turn180 extends InstantCommandBase{
    Swerve s_swerve;
    Intake i_intake;
    public Turn180(Intake m_intake){
        i_intake = m_intake;
        }
    @Override
    public void run(){
        // s_swerve.setHeading(new Rotation2d(180));
        i_intake.setspeed(.2);
    }

}

public class AutoTwoNoteCenter implements MCRCommand{    
    boolean first = true;
    double armMovementTimeout = .5;
    MCRCommand twoNoteCenter;
    Turn180 turn180;
    
    public AutoTwoNoteCenter(Swerve m_swerve, Intake i_intake, Shooter s_shooter, FullArmSubsystem a_arm){
        
        turn180 = new Turn180(i_intake);

        // auto variables
            // if(m_swerve==null) m_swerve = new Swerve();
            // if(i_intake==null) i_intake = new Intake();
            // if(s_shooter==null) s_shooter = new Shooter();
            // if(a_arm==null) a_arm = new FullArmSubsystem();
            
            // twoNoteCenter = new SequentialCommands(
            // // Auto Set Up
           
            
            // new InstantCommand(() -> m_swerve.resetModulesToAbsolute()),

            // // setting up speaker angles
            // new ParallelCommands(
            //     new InstantCommand(() -> a_arm.setSpeakerPosition()),
            //     new WaitCommand(armMovementTimeout)
            // ),
            // // shooting into the speaker
            // new InstantCommand(() -> s_shooter.setShootingSpeed()),
            // new WaitCommand(0.5),
            // new InstantCommand(() -> s_shooter.setStopSpeed()),

            // // // returning to stow
            // // new ParallelRaceGroup(
            // //     new ArmToAngles(m_wristSubsystem, m_ArmSubsystem, 0, 0),
            // //     new WaitCommand(armMovementTimeout)
            // // ),

            // // Pick up Floor note process
            // new parallelCommads(
            // // running the intake for picking the note up from ground
            // new SequentialCommands(
            // new InstantCommand(() -> i_intake.setIntakeTrue()),
            // new WaitCommand(0.5),
            // new InstantCommand(() -> i_intake.setIntakeFalse())
            // ), 
            // // sets the angles to the ground intake
            // new SequentialCommands(
            //     new ParallelCommands(
            //         new InstantCommand(() -> a_arm.setPickupPosition()),
            //         new WaitCommand(armMovementTimeout)
            //         ),
            //     new InstantCommand(() -> s_shooter.setShootingSpeed())
            //     ),
            // // drive to that note 
            // new SequentialCommands(
            //     new ParallelCommands(
            //         new InstantCommand(() -> m_swerve.driveToPoint(2.52, 5.54, 0)),
            //         new WaitCommand(3)
            //         )
            //         // new InstantCommand(() -> m_swerve.driveToPoint(-5.4, -0.38, 0))
            //     )
            // ),

            // // check or test before confirming this line
            // new InstantCommand(() -> i_intake.setIntakeFalse()
            // ),

            // //Arm angles to shooting position - speaker
            //     new ParallelCommands(
            //         new InstantCommand(() -> a_arm.setSpeakerPosition()),
            //         new WaitCommand(armMovementTimeout)
            //     ),

            // //shoot note into speaker
            // new InstantCommand(() -> s_shooter.setShootingSpeed()),
            // new WaitCommand(0.5),
            // new InstantCommand(() -> s_shooter.setStopSpeed()),

            // // //Arm angles to stow after shooting
            // new ParallelCommands(
            //     new InstantCommand(() -> a_arm.setRestPosition()),
            //     new WaitCommand(armMovementTimeout)
            // )
            // );
    }
    
    @Override
    public void run(){
        turn180.run();
        // LED.runOrange();
    }
    
    @Override
    public boolean isFinished(){
        return turn180.isFinished();
    }
}
