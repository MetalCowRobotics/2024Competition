// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib14.MCRCommand;
import frc.robot.autos.ArmToAngles2;
import frc.robot.subsystems.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.pathplanner.lib.auto.NamedCommands;

/*
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {
  public static final CTREConfigs ctreConfigs = new CTREConfigs();

    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final XboxController operator = new XboxController(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;
  
    private final Trigger crawl = new Trigger(() -> driver.getRawAxis(XboxController.Axis.kLeftTrigger.value) > 0.8);
    private final Trigger sprint = new Trigger(() -> driver.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.8);
    private final double shootervalue = XboxController.Axis.kRightTrigger.value;
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);

    /* Operator Controls */
    private final Trigger intakePosition = new Trigger(() -> operator.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.8);
    private final Trigger shooterPosition = new Trigger(() -> operator.getRawAxis(XboxController.Axis.kLeftTrigger.value) > 0.8);
    public boolean intakeStatus = false;

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final NoteTransitSubsystem m_NoteTransitSubsystem = NoteTransitSubsystem.getInstance();
    
    /* autos */
    MCRCommand autoMission;
    MCRCommand twoNoteCenter;

  /*
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
 
  @Override
  public void robotInit() {
    SmartDashboard.putNumber("Shooter Far Target", Constants.JointConstants.shooterFar);
    AutoBuilder.configureHolonomic(
            s_Swerve::getPose,
            s_Swerve::resetPose,
            s_Swerve::getRobotRelativeSpeeds,
            s_Swerve::driveRobotRelative,
            new HolonomicPathFollowerConfig(
                new PIDConstants(0.0, 0.0, 0.02),
                new PIDConstants(0.031, 0.0, 0.0),
                1.5,
                0.4,
                new ReplanningConfig(true,true)
            ),
            () -> {
                var alliance = DriverStation.getAlliance();
                return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
            },
             s_Swerve
        );
          NamedCommands.registerCommand("Shoot far Pos", new ArmToAngles2("speakerFromNote"));
          NamedCommands.registerCommand("rest Pos", new ArmToAngles2("rest"));
          NamedCommands.registerCommand("Shoot Pos", new ArmToAngles2("speaker"));
          NamedCommands.registerCommand("Intake Pos", new ArmToAngles2("pickup"));
          NamedCommands.registerCommand("Toggle Shooter", new InstantCommand(() -> m_NoteTransitSubsystem.toggleShooter()));
          NamedCommands.registerCommand("Intake Run", new InstantCommand(() -> m_NoteTransitSubsystem.enableIntake()));
          NamedCommands.registerCommand("Intake Stop", new InstantCommand(() -> m_NoteTransitSubsystem.disableIntake()));
     // Build an auto chooser. This will use Commands.none() as the default option.
    SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser("Center And Left");

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  @Override
  public void robotPeriodic() {
    s_Swerve.periodicValues();
    SmartDashboard.putNumber("Shooter Value", shootervalue);
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    PathPlannerAuto autoCommand = new PathPlannerAuto("Center And Left");
    autoCommand.schedule();
    System.out.println("Autonomous command scheduled");
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putString("hy", "5");
    s_Swerve.periodicValues();
    callPeriodic(); 
  }

  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();
    m_NoteTransitSubsystem.stopShooter();
    m_NoteTransitSubsystem.setRestPosition();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    configureButtonBindings();
    callPeriodic();

    s_Swerve.teleopSwerve(
      () -> -driver.getRawAxis(translationAxis), 
      () -> -driver.getRawAxis(strafeAxis), 
      () -> -driver.getRawAxis(rotationAxis), 
      () -> false /* Never Robot-Oriented */
    );
  }

  @Override
  public void testInit() {}
  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  private void configureButtonBindings() {
    /* Driver Related */
    if (zeroGyro.getAsBoolean()) {
      s_Swerve.zeroGyro();
      // if the Y button is pressed, the gyro will reset
    }

    if (crawl.getAsBoolean()) {
      s_Swerve.setCrawl();
      // if the left trigger is pressed, the robot will crawl
    }
    else if (sprint.getAsBoolean()) {
      s_Swerve.setSprint();
      // if the right trigger is pressed, the robot will sprint
    }
    else {
      s_Swerve.setBase();
      // if neither the left trigger or the right trigger is pressed, the robot will be in the base state
    }

    /* Operator Related */
    if (operator.getAButtonReleased()) {
      m_NoteTransitSubsystem.setRestPosition();
      // if Button A is released, the arm and wrist will go to the rest position
    }

     if (operator.getBButtonReleased()) {
      m_NoteTransitSubsystem.setStageShootingPosition();
      // if Button X is released, the arm and wrist will go to the climb final position
    }

    if (operator.getLeftBumperReleased()) {
      m_NoteTransitSubsystem.toggleShooter();
      // if the left bumper is released, the arm and wrist will go to the speaker position
    }

    if (intakeToggle()) {
      m_NoteTransitSubsystem.enableIntake();
      LED.runDefault();
    }
    else if (operator.getBackButton()) {
      m_NoteTransitSubsystem.quickOuttake();
    }
    else {
      m_NoteTransitSubsystem.disableIntake();
    }

    if (operator.getStartButtonReleased()) {
      m_NoteTransitSubsystem.setAMPPosition();
    }

    if (shooterPosition.getAsBoolean()) {
      m_NoteTransitSubsystem.setSpeakerPosition();
    }
    if (intakePosition.getAsBoolean()) {
      m_NoteTransitSubsystem.setPickupPosition();
    }
  }

    public void callPeriodic(){
      m_NoteTransitSubsystem.periodic();
    }

    public boolean intakeToggle(){
      if(m_NoteTransitSubsystem.getShootingState()){
        return operator.getRightBumperReleased();  
      }else{
        return operator.getRightBumper();
      }
    }
  }