package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class IntakeSubsystem {
    private static IntakeSubsystem instance = new IntakeSubsystem();
    private CANSparkMax intakeMotor;
    private boolean intakeEnabled;
    private double speed = 0;
    private DigitalInput intakeSensor;
    private boolean alreadyStopped;
    private boolean backOutPressed;
    // private double expectedTime = 0.18;
    private boolean retractReady = false;
    private boolean driving = false;
    //private boolean notePresent = false; 

    private IntakeSubsystem() {
        intakeMotor = new CANSparkMax(15, CANSparkLowLevel.MotorType.kBrushless);
        intakeMotor.setInverted(true);
        intakeSensor = new DigitalInput(1);//TODO: Put actual channel into the code
        intakeEnabled = false;
        alreadyStopped = false;
        SmartDashboard.putBoolean("IntakeEnabled", intakeEnabled);

    }

    public static IntakeSubsystem getInstance(){
        return instance;
    }



    public void periodic(){
       if(noteAcquired() && !alreadyStopped){
            stopintake();
            LED.runOrange();
            alreadyStopped = true;
       }else{
            LED.runDefault();
        }
        if(intakeEnabled)
            intakeMotor.set(speed);
        else{
            intakeMotor.set(0);
        }
        SmartDashboard.putNumber("IntakeSpeed", speed);
    }

    public boolean noteAcquired(){
        return !intakeSensor.get();
    }

    private void setStopDriving(){
        driving = true;
    }

    public void setStartDriving(){
        driving = false;
    }

    public boolean getStopDriving(){
        return driving;
    }

    public void setRetractReady(boolean b){
        retractReady = b;
    }

    public boolean getRetractReady(){
        return retractReady;
    }

    public void startIntake(){
        intakeEnabled = true;
    }

    public void startIntakeReverse(){
        speed = 0.85;
        intakeEnabled = true;
        backOutPressed = true;
    }

    public void setAlreadyStopped(boolean val){
        alreadyStopped = val;
    }

    public void stopintake(){
        intakeEnabled = false;
        backOutPressed = false;
    }

    public void setPickupSpeed(){
        speed = -.9;
    }

    public void setFeedSpeed(){
        speed = 1.0;
    }

    public void setAmpSpeed(){
        speed = 1.0;
    }

}