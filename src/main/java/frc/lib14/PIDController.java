package frc.lib14;

// calculate the error
// Error = Setpoint Value - Current Value
// determine the adjusted speeds of the motors.
// MotorSpeed = Kp * Error + Kd * ( Error - LastError );
// LastError = Error;
// RightMotorSpeed = RightBaseSpeed + MotorSpeed;
// LeftMotorSpeed = LeftBaseSpeed - MotorSpeed;
public class PIDController {
	// must experiment to get these right
	private double kP = 0;
	private double kI = 0;
	private double kD = 0;
	private double Iz = 0;

	// control variables
	private double setPoint;
	private double previousError = 0;
	private double accumulatedError = 0;

	public PIDController(double setPoint, double kP, double kI, double kD) {
		this.setPoint = setPoint;
		this.kD = kD;
		this.kI = kI;
		this.kP = kP;
	}

	public PIDController(double setPoint, double kP, double kI, double kD, double Iz) {
		this.setPoint = setPoint;
		this.kD = kD;
		this.kI = kI;
		this.kP = kP;
		this.Iz = Iz;
	}

	public double calculateAdjustment(double curPosition) {
		double error = calaculateError(setPoint, curPosition);
		double motorAdjustment = determineAdjustment(error);
		System.out.println("error: "+error+"  prev: "+previousError+"  accum: "+accumulatedError);
		previousError = error;
		System.out.println("Iz:"+Iz+"accumulated:"+accumulatedError);
		if (0 != Iz) {
			if (Math.abs(setPoint - curPosition) < Iz) {
				accumulatedError += error;	
				System.out.println("Accumulate IZ");
			} else {
				accumulatedError = 0;
				System.out.println("reset accumulated IZ");
			}
		}
		return motorAdjustment;
	}

	private double calaculateError(double setPoint, double curPosition) {
		return setPoint - curPosition;
	}

	private double determineAdjustment(double currentError) {
		return (kP * currentError) + (kD * (currentError - previousError)) + (kI * accumulatedError);
	}

	public double getSetPoint() {
		return setPoint;
	}

	public double getError() {
		return previousError;
	}

	public void setSetPoint(double setPoint) {
		this.setPoint = setPoint;
		reset();
	}

	public void set_kP(double kP) {
		this.kP = kP;
	}

	public void set_kI(double kI) {
		this.kI = kI;
	}


	public void set_kD(double kD) {
		this.kD = kD;
	}

	
	public void set_Iz(double Iz) {
		this.Iz = Iz;
	}

	public void reset() {
		previousError = 0;
		accumulatedError = 0;
	}

}
