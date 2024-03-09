package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
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
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.ArmToAngles;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class AutoOneNoteCenter implements MCRCommand{
    MCRCommand twoNoteCenter;
    Swerve m_swerve = new Swerve();
    Intake i_intake = new Intake();
    Shooter s_shooter = new Shooter();
    FullArm m_FullArmSubsytem = new FullArmSubsystem()

    public AutoOneNoteLeft(){
        new SequentialCommands(
        new ArmtoAngles(m_FullArmSubsystem, "speaker"),
        new StartIntake(m_Intake),
        new setShootingSpeed(s_shooter),
        new WaitCommand(.5),
        new setStopSpeed(s_shooter),
        new StopIntake(m_Intake),
        new StopIntake(m_Intake),
        new ArmToAngles(m_FullArmSubsystem, "rest"),        
        );
    
        @Override
        public void run(){
        }
        @Override boolean isFinished(){
            return false
        }
    }
}


