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


public class AutoTwoNoteCenter extends Command{    
    Swerve m_swerve;
    Intake i_intake;
    Shooter s_shooter; 
    FullArmSubsystem a_arm;
    double armMovementTimeout; 
    Command twoNoteCenter;
    public void TwoNoteInit(){
        m_swerve = new Swerve();
        // i_intake = new Intake();
        s_shooter = new Shooter();;
        a_arm= new FullArmSubsystem();
        armMovementTimeout = .5; 
    }
    
    
    public void twoNoteCenter(){
        // auto variables

            new SequentialCommandGroup(
            //Auto Set Up
            // new InstantCommand(() -> m_swerve.setHeading(new Rotation2d(0))),
            // new InstantCommand(() -> m_swerve.resetModulesToAbsolute()),

            // setting up speaker angles
            new ParallelRaceGroup(
                new InstantCommand(() -> a_arm.setSpeakerPosition()),
                new WaitCommand(armMovementTimeout)
            ),
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
            // new ParallelCommandGroup(
            // // running the intake for picking the note up from ground
            // new SequentialCommandGroup(
            // new InstantCommand(() -> i_intake.setIntakeTrue()),
            // new WaitCommand(0.5),
            // new InstantCommand(() -> i_intake.setIntakeFalse())
            // ), 
            // // sets the angles to the ground intake
            // new SequentialCommandGroup(
            //     new ParallelRaceGroup(
            //         new InstantCommand(() -> a_arm.setPickupPosition()),
            //         new WaitCommand(armMovementTimeout)
            //         ),
            //     new InstantCommand(() -> s_shooter.setShootingSpeed())
            //     ),
            // // drive to that note 
            // new SequentialCommandGroup(
            //     new ParallelRaceGroup (
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
            //     new ParallelRaceGroup(
            //         new InstantCommand(() -> a_arm.setSpeakerPosition()),
            //         new WaitCommand(armMovementTimeout)
            //     ),

            // //shoot note into speaker
            // new InstantCommand(() -> s_shooter.setShootingSpeed()),
            // new WaitCommand(0.5),
            // new InstantCommand(() -> s_shooter.setStopSpeed()),

            // //Arm angles to stow after shooting
            new ParallelRaceGroup(
                new InstantCommand(() -> a_arm.setRestPosition()),
                new WaitCommand(armMovementTimeout)
            )
        );
    }
}
