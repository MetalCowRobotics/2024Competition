// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib14.MCRCommand;
import frc.robot.autos.DriveOutAuto;
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
    private final JoystickButton visionAlignment = new JoystickButton(driver, XboxController.Button.kB.value);
  
    /* Operator Controls */
    private final Trigger intakePosition = new Trigger(() -> operator.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.8);
    private final Trigger shooterPosition = new Trigger(() -> operator.getRawAxis(XboxController.Axis.kLeftTrigger.value) > 0.8);
    public boolean intakeStatus = false;
    
    // private final JoystickButton intakeButton = new JoystickButton(operator, XboxController.Button.kB.value);
    // private final Trigger shooterTrigger = new Trigger(() -> operator.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.8);
    // private final JoystickButton armWrist = new JoystickButton(operator, XboxController.Button.kA.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Intake m_Intake = new Intake();
    private final Shooter m_Shooter = new Shooter();
    private final FullArmSubsystem m_FullArmSubsystem = new FullArmSubsystem();
    
    /* autos */
    MCRCommand autoMission;

  /*
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  @Override
  public void robotInit() {
    SmartDashboard.putNumber("Arm Kp", 0.06);
    SmartDashboard.putNumber("Arm Ki", 0.0);
    SmartDashboard.putNumber("Arm Kd", 0.012);

    SmartDashboard.putNumber("Intake Offset", 0);

    SmartDashboard.putNumber("AutoSelect",0);

    SmartDashboard.putNumber("Wrist Kp", 0.04);
    SmartDashboard.putNumber("Wrist Ki", 0.0);
    SmartDashboard.putNumber("Wrist Kd", 0.002);

    SmartDashboard.putNumber("StageArmAngleOffSet", 0);
    SmartDashboard.putNumber("AMPWristAngleOffSet", 0);
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
    s_Swerve.visionToGyro();
    s_Swerve.zeroGyro();
    s_Swerve.setHeading(new Rotation2d(Math.PI));
    // autoMission = new AutoTwoNoteCenter(s_Swerve, m_Intake, m_Shooter, m_FullArmSubsystem);
    //autoMission = new ShootNoteAuto(s_Swerve, m_Intake, m_Shooter, m_FullArmSubsystem);
    autoMission = new DriveOutAuto(s_Swerve, m_Intake);
    SmartDashboard.putString("auto", "stopped");
    
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    autoMission.run(); 
    callPeriodic(); 
  }

  @Override
  public void teleopInit() {
    s_Swerve.visionToGyro();
    m_FullArmSubsystem.setRestPosition();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    configureButtonBindings();
    callPeriodic();
    
    if(m_Intake.getRetractReady()){
      m_FullArmSubsystem.setRestPosition();
      m_Intake.resetNoteDetected();
      m_Intake.setRetractReady(false);
    }

    s_Swerve.periodic(
      () -> -driver.getRawAxis(translationAxis), 
      () -> -driver.getRawAxis(strafeAxis), 
      () -> -driver.getRawAxis(rotationAxis), 
      () -> false /* Never Robot-Oriented */
    );
    m_FullArmSubsystem.periodic();
    m_Intake.periodic();
    m_Shooter.periodic();
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

    if (visionAlignment.getAsBoolean()) {
      s_Swerve.enableVisionControl();
    }
    else {
      s_Swerve.disableVisionControl();
    }

    /* Operator Related */
    if (operator.getAButtonReleased()) {
      m_FullArmSubsystem.setRestPosition();
      m_Intake.stopintake();
    }

    if (operator.getYButtonReleased()) {
      m_FullArmSubsystem.setClimbVertPosition();
    }

    if (operator.getXButtonReleased()) {
      m_FullArmSubsystem.setClimbFinPosition();
    }

     if (operator.getBButtonReleased()) {
      m_FullArmSubsystem.setStageShootingPosition(SmartDashboard.getNumber("StageArmAngleOffSet", 0));
    }

    if (operator.getLeftBumper()) {
      m_Shooter.setShootingSpeed();
    } else {
      m_Shooter.setStopSpeed();
    }

    if (operator.getBackButton()) {
      m_Intake.startIntakeReverse();
    }
    else {
      m_Intake.stopintake();
    }

    if (operator.getRightBumperReleased()) {
      if (!intakeStatus) {
        m_Intake.startIntake();
        LED.runDefault();
        intakeStatus = true;
      } else {
        m_Intake.stopintake();
        intakeStatus = false;
      }
    }

    if (operator.getStartButtonReleased()) {
      m_FullArmSubsystem.setAMPPosition(SmartDashboard.getNumber("AMPWristAngleOffSet", 0));
    }

    if (shooterPosition.getAsBoolean()) {
      m_FullArmSubsystem.setSpeakerPosition();
    }
    if (intakePosition.getAsBoolean()) {
      m_FullArmSubsystem.setPickupPosition();
    }
  }

    public void callPeriodic(){
      m_FullArmSubsystem.periodic();
      m_Intake.periodic();
      m_Shooter.periodic();
    }
  }