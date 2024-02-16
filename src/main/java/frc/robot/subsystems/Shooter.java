package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

public class Shooter {
   private CANSparkMax shooterMotor1;   
   private CANSparkMax shooterMotor2;   
   private double motorSpeed = 0;

    public Shooter() {
        shooterMotor1 = new CANSparkMax(15, CANSparkLowLevel.MotorType.kBrushless);
        shooterMotor2 = new CANSparkMax(19, CANSparkLowLevel.MotorType.kBrushless);
        shooterMotor1.setInverted(true);
        shooterMotor2.setInverted(false);
    }

    public void periodic() {
        shooterMotor1.set(motorSpeed);
        shooterMotor2.set(motorSpeed);
    }

    public void setShootingSpeed() {
        motorSpeed = 0.6;
    }
    
    public void setStopSpeed() {
        motorSpeed = 0;
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