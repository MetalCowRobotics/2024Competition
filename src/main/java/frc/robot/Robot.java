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
import frc.robot.autos.Turn;
import frc.robot.autos.StartShooter;
import frc.robot.autos.StopShooter;
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
    
    // private final JoystickButton intakeButton = new JoystickButton(operator, XboxController.Button.kB.value);
    // private final Trigger shooterTrigger = new Trigger(() -> operator.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.8);
    // private final JoystickButton armWrist = new JoystickButton(operator, XboxController.Button.kA.value);

    MCRCommand autoMission;

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Intake m_Intake = new Intake();
    private final Shooter m_Shooter = new Shooter();
    private final FullArmSubsystem m_FullArmSubsystem = new FullArmSubsystem();
    
    /* autos */
    MCRCommand twoNoteCenter;
    
    // private SendableChooser m_autoSelector = new SendableChooser<MCRCommand>();
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
    // testAuto = new TestAuto(s_Swerve, m_Intake, m_Shooter, m_FullArmSubsystem); 
    s_Swerve.zeroGyro();

    s_Swerve.setHeading(new Rotation2d(Math.PI));
    // autoMission = new AutoTwoNoteCenter(s_Swerve, m_Intake, m_Shooter, m_FullArmSubsystem);
    //autoMission = new ShootNoteAuto(s_Swerve, m_Intake, m_Shooter, m_FullArmSubsystem);
    autoMission = new DriveOutAuto(s_Swerve, m_Intake);
    SmartDashboard.putString("auto", "stopped");
    // autoTwoNoteCenter = new AutoTwoNoteCenter(s_Swerve, m_Intake, m_Shooter, m_FullArmSubsystem);
    
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // s_Swerve.driveToPoint(1, 1, s_Swerve.getGyroYaw().getDegrees());
    s_Swerve.periodicValues();
    autoMission.run(); 
    //  twoNoteCenter.run();
     callPeriodic(); 

    // s_Swerve.driveToPoint(1, 1, s_Swerve.getGyroYaw().getDegrees());
    //testMotor.set(.15);
    //SmartDashboard.putNumber("Current",pdp.getCurrent(6));
    //SmartDashboard.putNumber("Voltage",pdp.getVoltage());

    //autoMission.run();
  }

  @Override
  public void teleopInit() {
    m_FullArmSubsystem.setRestPosition();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //LED.runOrange();
    //LED.runDefault();
    configureButtonBindings();
    callPeriodic();
    
    // if(m_Intake.getRetractReady()){
    //   m_FullArmSubsystem.setRestPosition();
    //   m_Intake.setRetractReady(false);
    //   LED.runOrange();
    // }else{
    //   LED.runDefault();
    // }
    
    // m_ArmSubsystem.setTarget(SmartDashboard.getNumber("Wanted Arm Angle", 0));
    // m_ArmSubsystem.getWristAngle(m_WristSubsystem.getCurrentAngle());
    //m_ArmSubsystem.periodic();

    // m_WristSubsystem.setTarget(SmartDashboard.getNumber("Wanted Wrist Angle", 0))
    // m_WristSubsystem.getArmAngle(m_ArmSubsystem.getEncoder1CurrentAngle());
    //m_WristSubsystem.periodic();
 
    if(m_Intake.getRetractReady()){
      m_FullArmSubsystem.setRestPosition();
      m_Intake.resetNoteDetected();
      m_Intake.setRetractReady(false);
    }

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
      m_FullArmSubsystem.setRestPosition();
      m_Intake.stopintake();
      // if Button A is released, the arm and wrist will go to the rest position
    }

    if (operator.getYButtonReleased()) {
      m_FullArmSubsystem.setClimbVertPosition();
      // if Button Y is released, the arm and wrist will go to the climb vertical position
    }

    if (operator.getXButtonReleased()) {
      m_FullArmSubsystem.setClimbFinPosition();
      // if Button X is released, the arm and wrist will go to the climb final position
    }

     if (operator.getBButtonReleased()) {
      m_FullArmSubsystem.setStageShootingPosition(SmartDashboard.getNumber("StageArmAngleOffSet", 0));
      // if Button X is released, the arm and wrist will go to the climb final position
    }

    if (operator.getLeftBumper()) {
      m_Shooter.setShootingSpeed();
      // m_FullArmSubsystem.setPickupPosition();
      // if the left bumper is released, the arm and wrist will go to the speaker position
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
    // else {
    //   m_FullArmSubsystem.setRestPosition();
    // }
    

    // if (intakeTrigger.getAsBoolean()) {
    //   m_Intake.startIntake();
    //   // if the right trigger is pressed, the intake will intake
    // } else if (intakeBackwards.getAsBoolean()) {
    //   m_Intake.startIntakeReverse();
    //   // if the back button is pressed, the intake will outtake
    // } else {
    //   m_Intake.stopintake();
    //   // if neither the right trigger or the back button is pressed, the intake will stop
    // }

    // if (shooterTrigger.getAsBoolean()) {
    //   m_Shooter.setShootingSpeed();
    //   // if the left trigger is pressed, the shooter will shoot
    // }
    // else {
    //   m_Shooter.setStopSpeed();
    //   // if the left trigger is not pressed, the shooter will stop
    // }
  }

    public void callPeriodic(){
      m_FullArmSubsystem.periodic();
      m_Intake.periodic();
      m_Shooter.periodic();
    }
  }