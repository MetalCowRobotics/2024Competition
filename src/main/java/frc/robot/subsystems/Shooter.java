package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter {
    private static Shooter instance = new Shooter();
    private CANSparkMax shooterMotor1;   
    private CANSparkMax shooterMotor2;  
    private RelativeEncoder shooterEncoder1;
    private RelativeEncoder shooterEncoder2;
    private double motorSpeed = 0;
    private boolean shooterEnabled;

    private Shooter() {
        shooterMotor1 = new CANSparkMax(17, CANSparkLowLevel.MotorType.kBrushless);
        shooterMotor2 = new CANSparkMax(52, CANSparkLowLevel.MotorType.kBrushless);
        shooterMotor1.setInverted(false);
        shooterMotor2.setInverted(true);
        shooterEncoder1 = shooterMotor1.getEncoder();
        shooterEncoder2 = shooterMotor2.getEncoder();
        shooterEnabled = false;
    }

    public static Shooter getInstance(){
        return instance;
    }

    public void periodic() {
        if(shooterEnabled){
            shooterMotor1.set(.9);
            shooterMotor2.set(.9);
        }
        SmartDashboard.putBoolean("Shooter Spun Up", getShooterSpunUp());
    }

    public boolean getShooterSpunUp(){
        if(((shooterEncoder1.getVelocity() / 5676) > 0.85) && ((shooterEncoder2.getVelocity() / 5676) > 0.85)){
            return true;        
        }
        return false;
    }

    public void toggleShooter(){
        if(shooterEnabled){
            shooterEnabled = false;
        }else{
            shooterEnabled = true;
        }
    }

    public void setShootingSpeed() {
        motorSpeed = 0.9;
    }
    
    public void setStopSpeed() {
        motorSpeed = 0;
    }
}