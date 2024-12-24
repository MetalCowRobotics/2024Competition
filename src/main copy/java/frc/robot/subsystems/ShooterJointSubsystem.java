package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import java.text.DecimalFormat;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
public class ShooterJointSubsystem {

    private static ShooterJointSubsystem instance = new ShooterJointSubsystem();
    private SparkMax shooterJointMotor;

    private PIDController pidController;

    private double maxSetpoint;
    private double minSetpoint;
    private double targetAngle;

    private DigitalInput boreInput;
    private DutyCycleEncoder boreEncoder;

    private double nominalVoltage = 12.6;
    private double rampTime = 0.250;
    private int stallCurrentLimit = 30;
    private int freeCurrentLimit = 30;
    private double maxRPM = 6200;
    private double minRPM = 0;
    private double kP = 0.04;
    private double kI = 0.0;
    private double kD = 0.00;
    private double positionTolerance = 2;
    private DecimalFormat dFormatter;
    private String distString;

    private ShooterJointSubsystem() {
        dFormatter = new DecimalFormat("#.#");
        shooterJointMotor = new SparkMax(17, SparkLowLevel.MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();

        config.voltageCompensation(nominalVoltage);

        config.openLoopRampRate(rampTime);
        config.closedLoopRampRate(rampTime);
        config.inverted(false);
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(stallCurrentLimit, freeCurrentLimit);

        shooterJointMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
        boreInput = new DigitalInput(2);
        boreEncoder = new DutyCycleEncoder(boreInput);
        maxSetpoint = maxRPM / 5820;
        minSetpoint = minRPM / 5820;
        pidController = new PIDController(kP, kI, kD);
        pidController.setIntegratorRange(-0.65, 0.65);
        SmartDashboard.putNumber("VariableShootingOffset", 0);
        SmartDashboard.putNumber("ShooterJointkp", kP);
        SmartDashboard.putNumber("ShooterJointkp", kI);
        SmartDashboard.putNumber("ShooterJointkp", kD);
    }

    public static ShooterJointSubsystem getInstance(){
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
        return -((boreEncoder.get() * 360) - 150);
    }

    public void setTarget(double target) {
        this.targetAngle = target;
    }

    public double getTargetAngle() {
        return this.targetAngle;
    }

    public void setVariableAngle(double xdist){
        distString = (dFormatter.format(xdist)).toString();
        if(distString.length()<4){
            distString.concat("00000000000000");
        }
        SmartDashboard.putString("distString", distString);
        int index1 = Integer.parseInt(distString.substring(0,1));
        SmartDashboard.putNumber("index1", index1);
        int index2 = Integer.parseInt(distString.substring(2,3));
        SmartDashboard.putNumber("index2", index2);
        setTarget((Constants.JointConstants.variableShootingConstants[index1][index2])+(SmartDashboard.getNumber("VariableShootingOffset",0)));
    }

    public void setVariableAngle2(double xdist){
        int index1 = (int)xdist;
        int index2 = (int)((xdist-(double)index1) * 10.0);
        SmartDashboard.putNumber("index1", index1);
        SmartDashboard.putNumber("index2", index2);
        setTarget((Constants.JointConstants.variableShootingConstants[index1][index2])+(SmartDashboard.getNumber("VariableShootingOffset",0)));
    }


    public double getVariableAngle(double xdist){
        distString = (dFormatter.format(xdist)).toString();
        return (Constants.JointConstants.variableShootingConstants[Integer.parseInt(distString.substring(0,1))][Integer.parseInt(distString.substring(2))]);
    }

    public boolean atTarget() {
        SmartDashboard.putBoolean("Shooter atTarget", Math.abs(this.targetAngle - getCurrentAngle()) < positionTolerance);
        SmartDashboard.putBoolean("Shooter atTarget", Math.abs(this.targetAngle - getCurrentAngle()) < positionTolerance);
        return Math.abs(targetAngle - getCurrentAngle()) < positionTolerance;
    }

    private void writeStatus() {
        SmartDashboard.putNumber("Shooter Angle", getCurrentAngle());
    }

    public void periodic() {
        writeStatus();



        //pidController.setPID(SmartDashboard.getNumber("ShooterJointkP", kP), SmartDashboard.getNumber("ShooterJointkI", kI), SmartDashboard.getNumber("ShooterJointkD", kD));
        double speed = 0;

        pidController.setSetpoint(targetAngle);

        if (!atTarget()) {
            speed = pidController.calculate(getCurrentAngle());
        }

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
        };
        SmartDashboard.putNumber("Shooter Joint Motor Output", speed);

        if (atTarget()) {
            shooterJointMotor.set(0);
        } else {
            shooterJointMotor.set(speed);
        }
    }
}
