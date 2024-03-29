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
    private double speed;
    private boolean shooterEnabled;

    private Shooter() {
        shooterMotor1 = new CANSparkMax(18, CANSparkLowLevel.MotorType.kBrushless);
        shooterMotor2 = new CANSparkMax(52, CANSparkLowLevel.MotorType.kBrushless);
        shooterMotor1.setInverted(true);
        shooterMotor2.setInverted(false);
        shooterEncoder1 = shooterMotor1.getEncoder();
        shooterEncoder2 = shooterMotor2.getEncoder();
        speed = 0;
        shooterEnabled = false;
    }

    public static Shooter getInstance(){
        return instance;
    }

    public void periodic() {
        if(shooterEnabled){
            shooterMotor1.set(speed);
            shooterMotor2.set(speed);
        } else {
            shooterMotor1.set(0);
            shooterMotor2.set(0);
        }
        SmartDashboard.putBoolean("Shooter Spun Up", getShooterSpunUp());
        SmartDashboard.putNumber("ShooterSide1Speed", shooterEncoder1.getVelocity());
        SmartDashboard.putNumber("ShooterSide2Speed", shooterEncoder1.getVelocity());

    }

    public boolean getShooterSpunUp(){
        if((speed == 1.0) && ((shooterEncoder1.getVelocity() > 4300.0) && (shooterEncoder2.getVelocity() > 4300.0))){
            return true;        
        }else if((speed == .5) && ((shooterEncoder1.getVelocity() > 2500.0) && (shooterEncoder2.getVelocity() > 2500.0))){
            return true;
        }
        return false;
    }

    public void setShootingSpeed(){
        speed = 1.0;
    }

    public void setAmpSpeed(){
        speed = .5;
    }

    public void toggleShooter(){
        if(shooterEnabled){
            shooterEnabled = false;
        }else{
            shooterEnabled = true;
        }
    }

}