// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.*;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static final CTREConfigs ctreConfigs = new CTREConfigs();

  /* Controllers */
    private final Joystick driver = new Joystick(0);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    private final Trigger crawl = new Trigger(() -> driver.getRawAxis(XboxController.Axis.kLeftTrigger.value) > 0.8);
    private final Trigger sprint = new Trigger(() -> driver.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.8);

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton playMusic = new JoystickButton(driver, XboxController.Button.kA.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
      
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    s_Swerve.musicInit();
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
  }

  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    configureButtonBindings();

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
    if (zeroGyro.getAsBoolean()) {
      s_Swerve.zeroGyro();
    }

    if (playMusic.getAsBoolean()) {
      s_Swerve.musicPlay();
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
  }
}
