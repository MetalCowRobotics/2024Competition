package frc.lib14;

// calculate the error
// Error = Setpoint Value - Current Value
// determine the adjusted speeds of the motors.
// MotorSpeed = Kp * Error + Kd * ( Error - LastError );
// LastError = Error;
// RightMotorSpeed = RightBaseSpeed + MotorSpeed;
// LeftMotorSpeed = LeftBaseSpeed - MotorSpeed;
public class PDController {
	// must experiment to get these right
	private double kP = .4;
	private double kD = .1;

	// control variables
	private double setPoint;
	private double previousError = 0;
	private double accumulatedError = 0;

	public PDController(double setPoint) {
		this.setPoint = setPoint;
	}

	public PDController(double setPoint, double Kp, double Kd) {
		this.setPoint = setPoint;
		this.kD = Kd;
		this.kP = Kp;
	}

	public double calculateAdjustment(double currentAngle) {
		double currentError = calaculateError(setPoint, currentAngle);
		double motorAdjustment = determineAdjustment(currentError, previousError);
		previousError = currentError;
		accumulatedError=accumulatedError+currentError;
		return motorAdjustment;
	}

	private double calaculateError(double setPoint, double curPosition) {
		return setPoint - curPosition;
	}

	private double determineAdjustment(double currentError, double previousError) {
		return kP * currentError + kD * (currentError - previousError);
		// return kP * currentError + kD * (currentError - previousError) + .00001 * accumulatedError;
	}

	public double getSetPoint() {
		return setPoint;
	}

	public double getError() {
		return previousError;
	}

	public void setSetPoint(double setPoint) {
		this.setPoint = setPoint;
	}

	public void set_kP(double kP) {
		this.kP = kP;
	}
	
	public void set_kD(double kD) {
		this.kD = kD;
	}
	
	public void reset() {
		previousError = 0;
		accumulatedError = 0;
	}

}
