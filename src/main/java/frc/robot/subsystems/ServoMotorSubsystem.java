package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class ServoMotorSubsystem extends SubsystemBase {
    private CANSparkMax m_motor;
    private RelativeEncoder encoder;

    private PIDController m_pidController;

    private double maxSetpoint;
    private double minSetpoint;

    private double setpoint;

    private double targetAngle;

    private double positionTolerance;

    private double initialPosition;

    private double reduction;

    private String subsystemName;

    public static class ServoMotorSubsystemConfig {
        public double nominalVoltage = 12.6;
        public double rampTime = 0.125;
        public int motorCanID;
        public boolean inverted = false;
        public CANSparkMax.IdleMode idleMode = CANSparkMax.IdleMode.kBrake;
        public int stallCurentLimit;
        public int freeCurentLimit;

        public double kP;
        public double kI;
        public double kD;

        public double positionTolerance;

        public double maxRPM;
        public double minRPM;

        public double initialPosition;

        public double reduction = 1;

        public String subsystemName;
    }

    protected ServoMotorSubsystem(ServoMotorSubsystemConfig config) {
        m_motor = new CANSparkMax(config.motorCanID, CANSparkMaxLowLevel.MotorType.kBrushless);
        m_motor.enableVoltageCompensation(config.nominalVoltage);
        // m_motor.disableVoltageCompensation();
        m_motor.setOpenLoopRampRate(config.rampTime);
        m_motor.setClosedLoopRampRate(config.rampTime);
        m_motor.setInverted(config.inverted);
        m_motor.setIdleMode(config.idleMode);
        m_motor.setSmartCurrentLimit(config.stallCurentLimit, config.freeCurentLimit);

        encoder = m_motor.getEncoder();
        // encoder.setPosition(0);
        //encoder.setInverted(config.inverted);

        maxSetpoint = config.maxRPM / 5820;
        minSetpoint = config.minRPM / 5820;

        this.reduction = config.reduction;

        m_pidController = new PIDController(config.kP, config.kI, config.kD);
        m_pidController.setIntegratorRange(-0.6, 0.6);

        this.positionTolerance = config.positionTolerance;

        this.initialPosition = config.initialPosition;

        this.subsystemName = config.subsystemName;
    }

    protected abstract boolean allowPositiveMotion(double angle);

    protected abstract boolean allowNegativeMotion(double angle);

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
        return Units.rotationsToDegrees(encoder.getPosition() / reduction) + initialPosition;
    }

    public void resetEncoder(double angle) {
        encoder.setPosition(angle);
    }

    public double getTargetAngle() {
        return this.targetAngle;
    }

    public double getCurrentVelocity() {
        return Units.rotationsToDegrees(encoder.getVelocity() / reduction);
    }

    public void setTarget(double target) {
        this.targetAngle = target;
        // this.setpoint = Units.degreesToRotations(target * reduction);
        m_pidController.setSetpoint(targetAngle);
    }

    public boolean atTarget() {
        return Math.abs(targetAngle - getCurrentAngle()) < positionTolerance;
    }

    private void writeStatus() {
        SmartDashboard.putNumber(subsystemName + " angle", getCurrentAngle());
        SmartDashboard.putNumber(subsystemName + " angular velocity", getCurrentVelocity());

        SmartDashboard.putNumber(subsystemName + " encoder position", encoder.getPosition());
        SmartDashboard.putNumber(subsystemName + " encoder velocity", encoder.getVelocity());

        SmartDashboard.putNumber(subsystemName + " target angle", targetAngle);
        SmartDashboard.putNumber(subsystemName + " target encoder podition", setpoint);
    }

    @Override
    public void periodic() {
        writeStatus();

        double speed = 0;

        if (!atTarget()) {
            speed = m_pidController.calculate(getCurrentAngle());
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
        }

        SmartDashboard.putNumber(subsystemName + " motor output", speed);
        if(atTarget()) {
            m_motor.set(0);
        } else {
            m_motor.set(speed);
        }
    }
}