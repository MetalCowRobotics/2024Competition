package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

public class Shooter {
   private CANSparkMax shooterMotor1;   
   private CANSparkMax shooterMotor2;   
   private double motorSpeed = 0;

    public Shooter() {
        shooterMotor1 = new CANSparkMax(17, CANSparkLowLevel.MotorType.kBrushless);
        shooterMotor2 = new CANSparkMax(52, CANSparkLowLevel.MotorType.kBrushless);
        shooterMotor1.setInverted(false);
        shooterMotor2.setInverted(true);
    }

    public void periodic() {
        shooterMotor1.set(motorSpeed);
        shooterMotor2.set(motorSpeed);
        // set the speed of the shooter motors to motorSpeed
    }

    public void setShootingSpeed() {
        motorSpeed = 0.9;
        // set the speed of the shooter motors to 0.9
    }
    
    public void setStopSpeed() {
        motorSpeed = 0;
        // set the speed of the shooter motors to 0
    }

    /* Methods for Commands */
    public void runShooter() {
        motorSpeed = 0.6;
        shooterMotor1.set(motorSpeed);
        shooterMotor2.set(motorSpeed);
    }

    public void stopShooter() {
        motorSpeed = 0;
        shooterMotor1.set(motorSpeed);
        shooterMotor2.set(motorSpeed);
    }
}