package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter {
    private static Shooter instance = new Shooter();
    private SparkMax shooterMotor1;   
    private SparkMax shooterMotor2;  
    private RelativeEncoder shooterEncoder1;
    private RelativeEncoder shooterEncoder2;
    private double speed;
    private boolean shooterEnabled;

    private Shooter() {
        shooterMotor1 = new SparkMax(18, SparkLowLevel.MotorType.kBrushless);
        shooterMotor2 = new SparkMax(52, SparkLowLevel.MotorType.kBrushless);

        SparkMaxConfig config1 = new SparkMaxConfig();
        SparkMaxConfig config2 = new SparkMaxConfig();

        config1.inverted(true);
        config2.inverted(false);

        shooterMotor1.configure(config1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shooterMotor2.configure(config2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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
            shooterMotor1.set(.0);
            shooterMotor2.set(.0);
        }
        SmartDashboard.putBoolean("Shooter Enabled", shooterEnabled);
        SmartDashboard.putBoolean("Shooter Spun Up", getShooterSpunUp());
        SmartDashboard.putNumber("ShooterSide1Speed", shooterEncoder1.getVelocity());
        SmartDashboard.putNumber("ShooterSide2Speed", shooterEncoder1.getVelocity());
    }
    
    public boolean getShooterSpunUp(){
        if((speed == 1.0) && ((shooterEncoder1.getVelocity() > 4000.0) && (shooterEncoder2.getVelocity() > 4000.0))){
            return true;        
        }
        return false;
    }

    public void setShootingSpeed(){
        speed = 1.0;
    }

    public void setAmpSpeed(){
        speed = .40;
    }

    public void toggleShooter(){
        if(shooterEnabled){
            shooterEnabled = false;
        }else{
            shooterEnabled = true;
        }
    }

    public void stopShooter() {
      shooterEnabled = false;
    }

    public void startShooter() {
       shooterEnabled = true;
    }
}