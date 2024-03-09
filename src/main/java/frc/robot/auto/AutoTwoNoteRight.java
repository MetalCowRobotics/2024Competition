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

public class AutoTwoNoteRight implements MCRCommand{
    MCRCommand twoNoteCenter;
    Swerve m_swerve = new Swerve();
    Intake i_intake = new Intake();
    Shooter s_shooter = new Shooter();
    FullArm m_FullArmSubsytem = new FullArmSubsystem()

    public AutoTwoNoteRight(){
        new SequentialCommands(
        new turn(m_swerve,45),
        new ArmToAngles(m_FullArmSubsystem, "speaker"),
        new StartIntake(m_Intake),
        new setShootingSpeed(s_shooter),
        new WaitCommand(armMovementTimeout),
        new StopIntake(m_Intake),
        new setStopSpeed(s_shooter),
        new ArmtoAngles(m_FullArmSubsystem, "rest"),
        new ParallelCommands(
            new ArmtoAngles(m_FullArmSubsystem, "pickup"),
            new StartIntake(m_Intake),
            new DriveToPointA(2.56, 6.90, 0.00),
        ),
        new StopIntake(m_Intake),
        new ArmToAngles(m_FullArmSubsystem, "speaker"),
        new turn(m_swerve, 55),
        new setShootingSpeed(s_shooter),
        new WaitCommand(.5),
        new setStopSpeed(s_shooter),
        );
        @Override
        public void run(){
        }
        @Override boolean isFinished(){
            return false
        }  
        
    }
}
