package frc.lib14;

import edu.wpi.first.wpilibj.XboxController;

public class XboxControllerMetalCow extends XboxController {

	public XboxControllerMetalCow(int port) {
		super(port);
	}

	public double getLT() {
		return this.getTriggerAxis(Hand.kLeft);
	}

	public double getRT() {
		return this.getTriggerAxis(Hand.kRight);
	}

	public double getLY() {
		return this.getY(Hand.kLeft);
	}

	public double getRY() {
		return this.getY(Hand.kRight);
	}

	public double getLX() {
		return this.getX(Hand.kLeft);
	}

	public double getRX() {
		return this.getX(Hand.kRight);
	}

	public boolean getLB() {
		return this.getBumper(Hand.kLeft);
	}

	public boolean getRB() {
		return this.getBumper(Hand.kRight);
	}

	public void rumbleLeft(double amt) {
		this.setRumble(RumbleType.kLeftRumble, amt);
	}

	public void rumbleRight(double amt) {
		this.setRumble(RumbleType.kRightRumble, amt);
	}
	
	public void rumbleAll(double amt) {
		rumbleLeft(amt);
		rumbleRight(amt);
	}

	public void rumbleNone() {
		rumbleLeft(0);
		rumbleRight(0);
	}

}
