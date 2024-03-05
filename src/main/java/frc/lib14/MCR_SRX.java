package frc.lib14;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.SpeedController;

public class MCR_SRX extends TalonSRX implements SpeedController {
	private double speedPercentage = 0;

	/**
	 * Set the output to the value calculated by PIDController.
	 *
	 * @param deviceNumber
	 *            - the CAN Device Number you will find in the RoboRio Dashboard at
	 *            10.42.13.2(wifi) or 172.22.11.2/(USB)
	 */
	public MCR_SRX(int deviceNumber) {
		super(deviceNumber);
	}

	/**
	 * Set the output to the value calculated by PIDController.
	 *
	 * @param output
	 *            the value calculated by PIDController
	 */
	@Override // SpeedController
	public void pidWrite(double output) {
		super.set(ControlMode.PercentOutput, output);
		this.speedPercentage = output;
	}

	/**
	 * Common interface for setting the speed of a speed controller.
	 *
	 * @param speed
	 *            The speed to set. Value should be between -1.0 and 1.0.
	 */
	@Override // SpeedController
	public void set(double speed) {
		super.set(ControlMode.PercentOutput, speed);
		this.speedPercentage = speed;
	}

	/**
	 * Common interface for getting the current set speed of a speed controller.
	 *
	 * @return The current set speed. Value is between -1.0 and 1.0.
	 */
	@Override // SpeedController
	public double get() {
		//the super method does not work
		//return super.getMotorOutputPercent();
		return this.speedPercentage;
	}

	/**
	 * Common interface for inverting direction of a speed controller.
	 *
	 * @param isInverted
	 *            The state of inversion true is inverted.
	 */
	@Override // SpeedController
	public void setInverted(boolean isInverted) {
		super.setInverted(isInverted);
	}

	/**
	 * Common interface for returning if a speed controller is in the inverted state
	 * or not.
	 *
	 * @return isInverted The state of the inversion true is inverted.
	 */
	@Override // SpeedController
	public boolean getInverted() {
		return super.getInverted();
	}

	/**
	 * Disable the speed controller.
	 */
	@Override // SpeedController
	public void disable() {
		super.neutralOutput();
		this.speedPercentage = 0;
	}

	/**
	 * Stops motor movement. Motor can be moved again by calling set without having
	 * to re-enable the motor.
	 */
	@Override // SpeedController
	public void stopMotor() {
		super.set(ControlMode.PercentOutput, 0); // Stop the motor
		this.speedPercentage = 0;
	}

}
