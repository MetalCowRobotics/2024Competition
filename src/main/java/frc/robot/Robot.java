// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.AutoTwoNoteCenter;
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
    private final JoystickButton playMusic = new JoystickButton(driver, XboxController.Button.kA.value);
  

    /* Operator Controls */
    /* Operator Controls */
    private final Trigger intakeTrigger = new Trigger(() -> operator.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.8);
    private final Trigger shooterTrigger = new Trigger(() -> operator.getRawAxis(XboxController.Axis.kLeftTrigger.value) > 0.8);
    // private final JoystickButton intakeButton = new JoystickButton(operator, XboxController.Button.kB.value);
    // private final Trigger shooterTrigger = new Trigger(() -> operator.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.8);
    // private final JoystickButton armWrist = new JoystickButton(operator, XboxController.Button.kA.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Intake m_Intake = new Intake();
    private final Shooter m_Shooter = new Shooter();
    // private final Intake m_Intake = new Intake();
    // private final Shooter m_Shooter = new Shooter();
    // private final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();
    // private final WristSubsystem m_WristSubsystem = new WristSubsystem();
    //private final RestToPickUp m_RestToShooter = new RestToPickUp(m_ArmSubsystem,m_WristSubsystem);
      private final FullArmSubsystem m_FullArmSubsystem = new FullArmSubsystem();
      private final AutoTwoNoteCenter a_AutoTwoNoteCenter = new AutoTwoNoteCenter();
    /* Commands */
    // private RestToShooter RestToShooter = new RestToShooter();
    //private InstantCommand ShooterToRest = new PickUpToRest();
    
  /*
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  @Override
  public void robotInit() {
    // s_Swerve.musicInit();
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
    s_Swerve.zeroGyro();

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    s_Swerve.driveToPoint(1, 1, s_Swerve.getGyroYaw().getDegrees());
    a_AutoTwoNoteCenter.periodic();
  }

  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    configureButtonBindings();
    m_Intake.periodic();
    m_Shooter.periodic();
      m_FullArmSubsystem.periodic();
    // m_ArmSubsystem.setTarget(SmartDashboard.getNumber("Wanted Arm Angle", 0));
    // m_ArmSubsystem.getWristAngle(m_WristSubsystem.getCurrentAngle());
    //m_ArmSubsystem.periodic();

    // m_WristSubsystem.setTarget(SmartDashboard.getNumber("Wanted Wrist Angle", 0))
    // m_WristSubsystem.getArmAngle(m_ArmSubsystem.getEncoder1CurrentAngle());
    //m_WristSubsystem.periodic();
 
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
    if (operator.getBButtonReleased()) {
      m_FullArmSubsystem.setPickupPosition();
    }
    if (operator.getAButtonReleased()) {
      m_FullArmSubsystem.setRestPosition();
      //System.out.println(m_FullArmSubsystem.getPhase());
    }
    if (operator.getYButtonReleased()) {
      m_FullArmSubsystem.setClimbVertPosition();
    }
    if (operator.getXButtonReleased()) {
      m_FullArmSubsystem.setClimbFinPosition();
    }
    if (operator.getLeftBumperReleased()) {
      m_FullArmSubsystem.setSpeakerPosition();
    }

    if (shooterTrigger.getAsBoolean()) {
      m_Shooter.setShootingSpeed();
    }
    else {
      m_Shooter.setStopSpeed();
    }

    if (intakeTrigger.getAsBoolean()) {
      m_Intake.setIntakeTrue();
    }
    else {
      m_Intake.setIntakeFalse();
    }
  }
    // else {
    //   m_FullArmSubsystem.setRestPosition();
    // }

    // if (armWrist.getAsBoolean()) {
    //   m_FullArmSubsystem.setIntakePosition();
    // }
    // else {
    //   m_FullArmSubsystem.setRestPosition();
    // }

    // if (armWrist2.getAsBoolean()) {
    //   m_FullArmSubsystem.move();
    // }
    // else {
    //   m_FullArmSubsystem.no();
    // }

    // if (shooterTrigger.getAsBoolean()) {
    //   m_Shooter.setShootingSpeed();
    // }
    // else {
    //   m_Shooter.setStopSpeed();
    // }

    // if (intakeButton.getAsBoolean()) {
    //   m_Intake.setIntakeTrue();
    // }
    // else {
    //   m_Intake.setIntakeFalse();
    // }

  }

