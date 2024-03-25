

package frc.lib14;

import edu.wpi.first.wpilibj.Joystick;

/**
 * Wrapper/convenience class for the Xbox360 gamepads that MetalCow has
 * 
 * Drivers through here http://wccftech.com/xbox-controller-pc-drivers-download-courtesy-major-nelson/
 * to here: mjr.mn/XboxOnePCDriversx64 if needed
 */

public class Xbox360Controller extends Joystick {
	
	public Xbox360Controller(int port) {
		super(port);
	}
    
    public double getLT(){
    		return getRawAxis(3);
    }
    public double getRT(){
    		return getRawAxis(4);
    }
	
	public double getLY() {
		return -getRawAxis(2);
	}
	
	public double getLX() {
		return getRawAxis(1);
	}
	
	public double getRY() {
		return -getRawAxis(6);
	}

	public double getRX() {
		return getRawAxis(5);
	}

	public boolean getButton(int button) {
		return getRawButton(button);
	}
	
	public void rumbleLeft(double amt){
		this.setRumble(RumbleType.kLeftRumble, amt);
	}
	public void rumbleRight(double amt){
		this.setRumble(RumbleType.kRightRumble, amt);
	}
	public void rumbleAll(double amt){
		rumbleLeft(amt);
		rumbleRight(amt);
	}
	public void rumbleNone(){
		rumbleLeft(0);
		rumbleRight(0);
	}
	
	
    public static class Button {
		public static final int A=1;
		public static final int B=2;
		public static final int X=3;
		public static final int Y=4;
		public static final int LB=5;
		public static final int RB=6;
		public static final int LT=7;
		public static final int RT=8;
		public static final int BACK=9;
		public static final int START=10;
		public static final int CENTER=11;
		public static final int DPADUP=12;
		public static final int DPADDOWN=13;
		public static final int DPADLEFT=14;
		public static final int DPADRIGHT=15;
    } 
}