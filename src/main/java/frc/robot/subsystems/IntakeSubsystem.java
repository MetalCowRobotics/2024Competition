package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

public class IntakeSubsystem {
    private static IntakeSubsystem instance = new IntakeSubsystem();
    private CANSparkMax intakeMotor;
    private boolean intakeEnabled;
    private boolean backOutPressed = false;
    private double speed = 0;
    private PowerDistribution pdp = new PowerDistribution(0,ModuleType.kCTRE);
    private DigitalInput intakeSensor;
    private Timer timer = new Timer();
    private Timer startUp = new Timer();
    // private double expectedTime = 0.18;
    private double expectedTime = 0.0;
    private boolean notedetected = false;
    private boolean retractReady = false;
    private boolean driving = false;
    //private boolean notePresent = false; 

    private IntakeSubsystem() {
        intakeMotor = new CANSparkMax(18, CANSparkLowLevel.MotorType.kBrushless);
        intakeMotor.setInverted(true);
        intakeSensor = new DigitalInput(0);//TODO: Put actual channel into the code
        intakeEnabled = false;
        SmartDashboard.putBoolean("IntakeEnabled", intakeEnabled);
    }

    public static IntakeSubsystem getInstance(){
        return instance;
    }

    // public void periodic() {
    //     if ((notedetected)) {
    //         LED.runOrange();
    //     }
    //     SmartDashboard.putNumber("startUp", startUp.get());
    //     SmartDashboard.putBoolean("Note detected", notedetected);
    //     SmartDashboard.putNumber("speed", speed);
    //     SmartDashboard.putNumber("Current",pdp.getCurrent(6));
    //     if(startUp.get() < 0.5){
    //         intakeMotor.set(speed);
    //         return;
    //     }
    //     if (noteAcquired()) {
    //         setRetractReady(true);
    //         stopintake();
    //         setStopDriving();
    //     }
    //     // if(speed >= 0){
    //     //     System.out.println("speed > 0");
    //     //     if (notePresent() && !notedetected) {
    //     //         notedetected = true;
    //     //         setRetractReady(true);
    //     //         timer.reset();
    //     //         timer.start();
    //     //     } 
    //     //     if (notedetected && timer.get() > expectedTime){
    //     //         stopintake();
    //     //         SmartDashboard.putString("auto", "stopped");
    //     //         // setStopDriving();
    //     //         timer.stop();
    //     //     }
    //     intakeMotor.set(speed);   
    // }

    public void periodic(){
        if(noteAcquired()){
            LED.runOrange();
            if(!backOutPressed){
                stopintake();
            }
        }else{
            LED.runDefault();
        }
        if(intakeEnabled)
            intakeMotor.set(speed);
    }

    // private boolean noteAcquired() {
    //     if (notePresent() && !notedetected) {
    //             notedetected = true;
    //             timer.reset();
    //             timer.start();
    //         } 
    //         if (notedetected && timer.get() >= expectedTime){
    //             SmartDashboard.putString("auto", "stopped");
    //             timer.stop();
    //             return true;
    //         }
    //     return false;
    // }

    public boolean noteAcquired(){
        return intakeSensor.get();
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

    // private boolean notePresent(){
    //     return pdp.getCurrent(6) > 15;
    // }

    public void resetNoteDetected(){
        notedetected = false;
        LED.runDefault();
    }

    public void setRetractReady(boolean b){
        retractReady = b;
    }

    public boolean getRetractReady(){
        return retractReady;
    }

    // public void startIntake(){
    //     if(intakeEnabled){
    //         intakeEnabled = false;
    //     }else{
    //         intakeEnabled = true;
    //     }
    // }

    public void startIntake(){
        intakeEnabled = true;
    }

    // public void startIntake(){
    //     startUp.reset();
    //     startUp.start();
    //     speed = 0.85;
    //     setRetractReady(false);
    //     notedetected = false;
        
    // }

    public void startIntakeReverse(){
        speed = -0.85;
        intakeEnabled = true;
        backOutPressed = true;
    }

    // public void feed(){
    //     startUp.reset();
    //     startUp.start();
    //     speed = 1;
    //     setRetractReady(false);
    //     notedetected = false;    }

    public void stopintake(){
        intakeEnabled = false;
        backOutPressed = false;
        speed = 0;
    }

    public void setPickupSpeed(){
        speed = .85;
    }

    public void setFeedSpeed(){
        speed = -1.0;
    }

    public void setAmpSpeed(){
        speed = -.85;
    }



}