package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.*;
import frc.lib14.MCRCommand;
import frc.lib14.SequentialCommands;
import frc.lib14.ParallelCommands;
import frc.lib14.InstantCommandBase;
import frc.robot.autos.SetShootingSpeed;


abstract class RotateToSpeaker extends InstantCommandBase{

    Swerve m_swerve;

    public RotateToSpeaker(Swerve swerve){
        m_swerve = swerve;
    }

    @Override
    public void run(){

        m_swerve.setHeading(new Rotation2d(135));
    }
    @Override
    public boolean isFinished(){return false;}
}

public class AutoTwoNotesLeft implements MCRCommand {
  /* Subsystems */
    public Swerve swerve;
    public Intake intake;
    public Shooter shooter;
    public Arm arm;
    public Wrist wrist;
    public RotateToSpeaker RotateRobot;

    public AutoTwoNotesLeft(Swerve swerve, Shooter shooter, Intake intake){}
    @Override
    public boolean isFinished(){return false;}
    @Override
    public void run(){}
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public void RobotContainer() {
        
        swerve = new Swerve();
        intake = new Intake();
        shooter = new Shooter();
        arm = new Arm();
        wrist= new Wrist();
        RotateRobot = new RotateToSpeaker(swerve);
        
        Rotation2d leftSpeakerHeading = new Rotation2d(135);

        SequentialCommands autoTwoNotesLeft = new SequentialCommands( 
            //new setHeading(leftSpeakerHeading)), // face the speaker from left
            RotateRobot.run(),
            swerve.resetModulesToAbsolute(),
            new SetShootingSpeed(swerve).run(),
            new TimedCommandSet(shooter.setStopSpeed(),0.5),
            //new InstantCommand(() -> shooter.setStopSpeed()),
            new InstantCommand(() -> swerve.zeroGyro()),
            new ParallelCommands(

                new InstantCommand(() -> swerve.driveToPoint(-4.0, 0.0, 180.0)),
                new InstantCommand(() -> intake.setIntakeTrue())
            ),
            new InstantCommand(() -> swerve.driveToPoint(-4.0, 0.0, 180.0))
            //new EnableVision(swerve)
        );   
}
}
