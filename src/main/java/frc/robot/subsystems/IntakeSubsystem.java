package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.CTREConfigs;

public class IntakeSubsystem extends SubsystemBase {
    private boolean debug = false;


    private TalonSRX intakeMotor = new TalonSRX(16);
    private double motorSpeed = 0;
    int i=0;
    CTREConfigs configs = new CTREConfigs();

    private DigitalInput noteSensor;
   

    private Debouncer noteDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kBoth);

    public IntakeSubsystem() {
        intakeMotor.configAllSettings(configs.intakeMotorConfig);
        intakeMotor.setNeutralMode(NeutralMode.Brake);
        
        DigitalInput noteSensor = new DigitalInput((int) IntakeConstants.NOTE_SENSOR_DIO);
        
    }


    public void setDebug(boolean debug) {
        this.debug = debug;
    }

    // note intake
    public void run(){
        motorSpeed = IntakeConstants.INTAKE_SPEED;
        IntakeConstants.intakeRunning = true;
    }

    // note eject
    public void runReverse() {
        motorSpeed = -IntakeConstants.INTAKE_SPEED;
        IntakeConstants.intakeRunning = true;
    }


    public void stop(){
        motorSpeed = 0;
        IntakeConstants.intakeRunning = false;
    }

    public boolean noteInIntake() {
        return !noteDebouncer.calculate(noteSensor.get());
    }

    



    @Override
    public void periodic() {
        SmartDashboard.putBoolean("note intake", noteInIntake());
        SmartDashboard.putBoolean("note sensor", noteSensor.get());
        intakeMotor.set(TalonSRXControlMode.PercentOutput, motorSpeed);
    }
}




