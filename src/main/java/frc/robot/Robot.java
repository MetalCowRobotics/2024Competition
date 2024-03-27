// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib14.MCRCommand;
import frc.lib14.SequentialCommands;
import frc.robot.autos.ArmToAngles;
import frc.robot.autos.AutoTwoNoteCenter;
import frc.robot.autos.DriveOutAuto;
import frc.robot.autos.DriveToPointA;
import frc.robot.autos.ResetModulesToAbsolute;
import frc.robot.autos.ShootNoteAuto;
import frc.robot.autos.StartIntake;
import frc.robot.autos.StopIntake;
// import frc.robot.autos.TestAuto;
import frc.robot.autos.ToggleShooter;
import frc.robot.subsystems.*;
import frc.lib14.*;
import com.revrobotics.CANSparkLowLevel;
/*
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {
  public static final CTREConfigs ctreConfigs = new CTREConfigs();
  private PowerDistribution pdp = new PowerDistribution(10,ModuleType.kCTRE);

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
    //LED.runOrange();
    // s_Swerve.musicInit();
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

    // SmartDashboard.
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
    if(SmartDashboard.getNumber("AutoSelect", 0) == 0){
      autoMission = new AutoTwoNoteCenter(s_Swerve);
    }
    // testAuto = new TestAuto(s_Swerve, m_Intake, m_Shooter, m_FullArmSubsystem); 
    s_Swerve.zeroGyro();

    s_Swerve.setHeading(new Rotation2d(Math.PI));

    SmartDashboard.putString("auto", "stopped");
    // autoTwoNoteCenter = new AutoTwoNoteCenter(s_Swerve, m_Intake, m_Shooter, m_FullArmSubsystem);
    
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // s_Swerve.driveToPoint(1, 1, s_Swerve.getGyroYaw().getDegrees());
    s_Swerve.periodicValues();
    autoMission.run(); 
    callPeriodic(); 

  }

  @Override
  public void teleopInit() {
    m_NoteTransitSubsystem.setRestPosition();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //LED.runOrange();
    //LED.runDefault();

    configureButtonBindings();
    SmartDashboard.putNumber("yawTeleOp", s_Swerve.getGyroYaw().getDegrees());
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