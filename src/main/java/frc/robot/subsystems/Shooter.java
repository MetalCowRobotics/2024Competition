package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter {
    private CANSparkMax shooterMotor1;   
    private CANSparkMax shooterMotor2;  
    private RelativeEncoder shooterEncoder1;
    private RelativeEncoder shooterEncoder2;
    private double motorSpeed = 0;

    public Shooter() {
        shooterMotor1 = new CANSparkMax(17, CANSparkLowLevel.MotorType.kBrushless);
        shooterMotor2 = new CANSparkMax(52, CANSparkLowLevel.MotorType.kBrushless);
        shooterMotor1.setInverted(false);
        shooterMotor2.setInverted(true);
        shooterEncoder1 = shooterMotor1.getEncoder();
        shooterEncoder2 = shooterMotor2.getEncoder();
    }

    public void periodic() {
        shooterMotor1.set(motorSpeed);
        shooterMotor2.set(motorSpeed);
        SmartDashboard.putBoolean("Shooter Spun Up", getShooterSpunUp());
    }

    public boolean getShooterSpunUp(){
        if(((shooterEncoder1.getVelocity() / 5676) > 0.85) && ((shooterEncoder2.getVelocity() / 5676) > 0.85)){
            return true;        
        }
        return false;
    }

    public void setShootingSpeed() {
        motorSpeed = 1;
    }
    
    public void setStopSpeed() {
        motorSpeed = 0;
    }
}