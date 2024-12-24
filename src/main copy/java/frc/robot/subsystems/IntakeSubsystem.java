package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeSubsystem {
    private static IntakeSubsystem instance = new IntakeSubsystem();
    private SparkMax intakeMotor;
    private boolean intakeEnabled;
    private double speed = 0;
    private DigitalInput intakeSensor;
    private boolean alreadyStopped;
    private boolean driving = false;
    private boolean readyToLift = false;
    private PowerDistribution pdp = new PowerDistribution(0,ModuleType.kCTRE);
    private boolean notedetected = false;
    private boolean retractReady = false;

    private IntakeSubsystem() {
        intakeMotor = new SparkMax(15, SparkLowLevel.MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();

        config.inverted(true);

        intakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        intakeSensor = new DigitalInput(1);
        intakeEnabled = false;
        alreadyStopped = false;
        SmartDashboard.putBoolean("Intake Enabled", intakeEnabled);
    }

    public static IntakeSubsystem getInstance(){
        return instance;
    }

    public void periodic(){
        // if(startUp.get() < 0.5){
        //     intakeMotor.set(speed);
        //     return;
        // }
        // if (noteAcquired()) {
        //     setRetractReady(true);
        //     stopintake();
        //     setStopDriving();
        // }
        // if(intakeEnabled)
        //     intakeMotor.set(speed);
        // else{
        //     intakeMotor.set(0);
        // }

       if(noteAcquired() && !alreadyStopped){
            stopintake();
            readyToLift = true;
            //intakeMotor.set(0);
            alreadyStopped = true;
       }
        if(intakeEnabled)
            intakeMotor.set(speed);
        else{
            intakeMotor.set(0);
        }
        SmartDashboard.putBoolean("Intake Enabled", intakeEnabled);
        SmartDashboard.putNumber("IntakeSpeed", speed);
        SmartDashboard.putBoolean("NoteAcquired", noteAcquired());
        SmartDashboard.putBoolean("AlreadyStopped", alreadyStopped);
    }

    public boolean noteAcquired(){
        return !intakeSensor.get();
    }
    // private boolean notePresent(){
    //     return pdp.getCurrent(6) > 15;
    // }

    // public boolean noteAcquired() {
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
    

    public void setStopDriving(){
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

    public void setReadyToLift(Boolean b){
        readyToLift = b;
    }

    public boolean getReadyToLift(){
        return readyToLift;
    }

    public boolean getRetractReady(){
        return retractReady;
    }

    public void startIntake(){
        // startUp.reset();
        // startUp.start();
        // speed = 0.9;
        notedetected = false;
        intakeEnabled = true;
    }

    public void toggleIntake(){
        if (intakeEnabled){
            intakeEnabled = false;
        }else{
            intakeEnabled = true;
        }
    }

    public void startIntakeReverse(){
        speed = 1;
        intakeEnabled = true;
    }

    public void feedIntakeAuto(){
        speed = 1;
        intakeEnabled = true;
    }

    public void setAlreadyStopped(boolean val){
        alreadyStopped = val;
    }

    public void stopintake(){
        intakeEnabled = false;
    }

    public void setPickupSpeed(){
        speed = -.95;
    }

    public void setFeedSpeed(){
        speed = 1.0;
    }

    public void setAmpSpeed(){
        speed = 1.0;
    }
}