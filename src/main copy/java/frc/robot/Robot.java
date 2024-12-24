
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib14.MCRCommand;
import frc.robot.subsystems.*;
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
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
  
    /* Operator Controls */
    private final Trigger intakePosition = new Trigger(() -> operator.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.8);
    private final Trigger shooterPosition = new Trigger(() -> operator.getRawAxis(XboxController.Axis.kLeftTrigger.value) > 0.8);
    public boolean intakeStatus = false;
    
    // private final JoystickButton intakeButton = new JoystickButton(operator, XboxController.Button.kB.value);
    // private final Trigger shooterTrigger = new Trigger(() -> operator.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.8);
    // private final JoystickButton armWrist = new JoystickButton(operator, XboxController.Button.kA.value);

    MCRCommand autoMission;

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final NoteTransitSubsystem m_NoteTransitSubsystem = NoteTransitSubsystem.getInstance();
    
    /* autos */
    MCRCommand twoNoteCenter;

      SendableChooser<Command> autoChooser ;

  /*
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

 
  @Override
  public void robotInit() {
    //              ||||||||               ||||||||               ||||||||   
    //              ||||||||               ||||||||               ||||||||
    //              ||||||||               ||||||||               ||||||||
    //              ||||||||               ||||||||               ||||||||
    //              ||||||||               ||||||||               ||||||||
    //              ||||||||               ||||||||               ||||||||
    //              ||||||||               ||||||||               ||||||||
    //              ||||||||               ||||||||               ||||||||
    //              ||||||||               ||||||||               ||||||||
    //              ||||||||               ||||||||               ||||||||
    //         \\\\\\||||||//////     \\\\\\||||||//////     \\\\\\||||||//////
    //          \\\\\\||||//////       \\\\\\||||//////       \\\\\\||||//////
    //           \\\\\\||//////         \\\\\\||//////         \\\\\\||//////  
    //            \\\\\\//////           \\\\\\//////           \\\\\\//////
    //             \\\\\/////             \\\\\/////             \\\\\/////
    //              \\\\////               \\\\////               \\\\////
    //               \\\///                 \\\///                 \\\/// 
    //                \\//                   \\//                   \\//
    //                 \/                     \/                     \/
    // IMPORTANT NOTE FOR AUTOS IF YOU MAKE AN AUTO THAT BREAKS IT IS ON THE ROBO RIO UNTELL YOU REFORMAT IT 
    // SO EVAN IF YOU FIX THE CODE IT WONT WORK TELL YOU REFORMAT THE ROBO RIO
    SmartDashboard.putNumber("Shooter Far Target", Constants.JointConstants.shooterFar);
  //   AutoBuilder.configureHolonomic(
  //           s_Swerve::getPose,
  //           s_Swerve::resetPose,
  //           s_Swerve::getRobotRelativeSpeeds,
  //           s_Swerve::driveRobotRelative,
  //           new HolonomicPathFollowerConfig(
  //               new PIDConstants(0.0, 0.0, 0.0),
  //               new PIDConstants(0.0, 0.0, 0.1),
  //               3,
  //               0.4,
  //               new ReplanningConfig(true,true)
  //           ),
  //           () -> {
  //               var alliance = DriverStation.getAlliance();
  //               return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
  //           },
  //            s_Swerve
  //       );
  //         NamedCommands.registerCommand("Shoot far Pos", new ArmToAngles2("speakerFromNotePosition"));
  //         // NamedCommands.registerCommand("Shoot mid Pos", new ArmToAngles2("speakerMidPosition"));
  //         NamedCommands.registerCommand("rest Pos", new ArmToAngles2("restPosition"));
  //         NamedCommands.registerCommand("Shoot Pos", new ArmToAngles2("speakerPosition"));
  //         NamedCommands.registerCommand("Intake Pos", new ArmToAngles2("pickupPosition"));
  //         NamedCommands.registerCommand("Toggle Shooter", new InstantCommand(() -> m_NoteTransitSubsystem.toggleShooter()));
  //         // NamedCommands.registerCommand("Toggle Intake", new InstantCommand(() -> m_NoteTransitSubsystem.toggleIntake()));
  //         // NamedCommands.registerCommand("Intake Feed", new InstantCommand(() -> m_NoteTransitSubsystem.quickOuttake()));
  //         // NamedCommands.registerCommand("Intake Stop", new InstantCommand(() -> m_NoteTransitSubsystem.disableIntake()));
  //         NamedCommands.registerCommand("Enable Intake", new InstantCommand(() -> m_NoteTransitSubsystem.enableIntake()));
  //    // Build an auto chooser. This will use Commands.none() as the default option.
  //   autoChooser = AutoBuilder.buildAutoChooser("Amp");

  // //   // Another option that allows you to specify the default auto by its name
  //   // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");
  //  // autoChooser =  AutoBuilder.buildAutoChooser("Red Left Three Note Auto");

  // SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  @Override
  public void robotPeriodic() {
    s_Swerve.periodicValues();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  
  public void autonomousInit() {
    Command autoCommand = autoChooser.getSelected();
     autoCommand.schedule();
    System.out.println("Autonomous command scheduled");
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    autoMission.run();
    CommandScheduler.getInstance().run();
    SmartDashboard.putString("hy", "5");
    s_Swerve.periodicValues();
    callPeriodic(); 
  }

  @Override
  public void teleopInit() {
    //s_Swerve.setDriveOffsets();
    CommandScheduler.getInstance().cancelAll();
    m_NoteTransitSubsystem.stopShooter();
    m_NoteTransitSubsystem.setRestPosition();
    s_Swerve.zeroGyro();
    if (DriverStation.getAlliance().equals(Alliance.Red)) {
      s_Swerve.setHeading(new Rotation2d(Math.toDegrees(Math.PI)));
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // System.out.println(s_Swerve.getTotalDist());

    configureButtonBindings();
    callPeriodic();
    s_Swerve.periodic(
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
    }

    if (crawl.getAsBoolean()) {
      s_Swerve.setCrawl();
    }
    else if (sprint.getAsBoolean()) {
      s_Swerve.setSprint();
    }
    else {
      s_Swerve.setBase();
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

    if (operator.getYButton()){
      // m_NoteTransitSubsystem.setVariableAngle(s_Swerve.getTotalDist());
      
    }

    if (operator.getLeftBumperButtonReleased()) {
      m_NoteTransitSubsystem.toggleShooter();
      // if the left bumper is released, the arm and wrist will go to the speaker position
    }

    if (operator.getRightBumperButtonReleased()) {
      m_NoteTransitSubsystem.enableIntake();
    }
    else if (operator.getBackButton()) {
      m_NoteTransitSubsystem.quickOuttake();
    }
    // else {
    //   m_NoteTransitSubsystem.disableIntake();
    // }

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

    // public boolean intakeToggle(){
    //   if(m_NoteTransitSubsystem.getShootingState()){
    //     return operator.getRightBumper();  
    //   }else{
    //     return operator.getRightBumper();
    //   }
    }
  