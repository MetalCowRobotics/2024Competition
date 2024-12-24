package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeJointSubsystem {
    private static IntakeJointSubsystem instance = new IntakeJointSubsystem();
    private SparkMax intakeJointMotor;

    private PIDController pidController;
    private double maxSetpoint;
    private double minSetpoint;
    private double targetAngle;

    private DigitalInput boreInput = new DigitalInput(0);
    private DutyCycleEncoder boreEncoder = new DutyCycleEncoder(boreInput);
    private double boreRawValue, boreConvertedValue, boreConvertedOffsetValue;

    private double nominalVoltage = 12.6;
    private double rampTime = 0.50;
    private int stallCurrentLimit = 30;
    private int freeCurrentLimit = 30;
    private double maxRPM = 1500;
    private double minRPM = 0;
    private double kP = 0.005; //0.005
    private double kI = 0.0;
    private double kD = 0.0001;
    private double positionTolerance = 6; 

    private IntakeJointSubsystem() {

        intakeJointMotor = new SparkMax(16, SparkLowLevel.MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();

        config.inverted(false);
        config.idleMode(IdleMode.kBrake);
        config.closedLoopRampRate(rampTime);
        config.voltageCompensation(nominalVoltage);
        config.openLoopRampRate(rampTime);
        config.smartCurrentLimit(stallCurrentLimit, freeCurrentLimit);

        intakeJointMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        maxSetpoint = maxRPM / 5820;
        minSetpoint = minRPM / 5820;

        pidController = new PIDController(kP, kI, kD);
        pidController.setTolerance(positionTolerance);
        pidController.setIntegratorRange(-0.65, 0.65);
    }

    public static IntakeJointSubsystem getInstance(){
        return instance;
    }

    private boolean allowPositiveMotion(double angle) {
        return angle >= 0;
    }

    private boolean allowNegativeMotion(double angle) {
         return angle <= 180;
    }

    private double limitSpeed(double speed) {
        if (speed == 0) {
            return 0.0;
        }
        double absSpeed = Math.abs(speed);
		absSpeed = Math.max(absSpeed, minSetpoint);
		absSpeed = Math.min(absSpeed, maxSetpoint);
		return Math.copySign(absSpeed, speed);
    }

    public double getCurrentAngle() {
        return boreConvertedOffsetValue;
    }

    public void setTarget(double target) {
        this.targetAngle = target;
    }

    public double getTargetAngle() {
        return this.targetAngle;
    }

    public boolean atTarget() {
        SmartDashboard.putBoolean("Intake atTarget", pidController.atSetpoint());
        return pidController.atSetpoint();
    }

    public boolean atAngle(double desiredAngle) {
        return Math.abs(desiredAngle - getCurrentAngle()) < positionTolerance;
    }
    

    private void writeStatus() {
        SmartDashboard.putNumber("Intake Angle", getCurrentAngle());
    }

    public void periodic() {
        boreRawValue = boreEncoder.get();
        boreConvertedValue = boreRawValue * (360);
        boreConvertedOffsetValue = (boreConvertedValue - 106.084371);
        writeStatus();
        
        double speed = 0;

        pidController.setPID(SmartDashboard.getNumber("IntakeJointkP", kP), SmartDashboard.getNumber("IntakeJointkI", kI), SmartDashboard.getNumber("IntakeJointkD", kD));

        pidController.setSetpoint(targetAngle);

        // if (!atTarget()) {
            speed = pidController.calculate(getCurrentAngle());
        // }

        speed = limitSpeed(speed);
        if (speed < 0) {
            if (!allowNegativeMotion(speed)) {
                speed = 0;
            }
        }

        if (speed > 0) {
            if (!allowPositiveMotion(speed)) {
                speed = 0;
            }
        }
        SmartDashboard.putNumber("Intake Joint Motor Output", speed);

        if (atTarget()) {
            intakeJointMotor.set(0);
        } else {
            intakeJointMotor.set(speed);
        }
    }
}
