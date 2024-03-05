package frc.lib14;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;

//this still depends on having two RobotMaps, one for competition and one for test
//and make sure they do not get clobbered in merges.
public class ComponentBuilder {
	public static enum MOTOR_TYPE {
		TALON, TALONSRX
	};

	// this is in RobotMap
	public final static int[] RightMotors = { 0, 1 };
	public final static int[] LeftMotors = { 3 };

	// this is in the system
	private void execute() {
		SpeedController right = buildMotorGroup(MOTOR_TYPE.TALON, RightMotors, true);
		SpeedController left = buildMotorGroup(MOTOR_TYPE.TALON, LeftMotors, false);
	}

	public static SpeedController buildMotor(MOTOR_TYPE type, int motorChannel) {
		try {
			switch (type) {
			case TALON:
				return new Talon(motorChannel);
			case TALONSRX:
				return new MCR_SRX(motorChannel);
			default:
				return null;
			}
		} catch (Exception ex) {
			System.err.println(ex.getMessage());
		}
		return null;
	}

	public static SpeedController buildMotor(MOTOR_TYPE type, int motorChannel, boolean reverseProperty) {
		SpeedController motor = buildMotor(type, motorChannel);
		if (motor != null) {
			motor.setInverted(reverseProperty);
		}
		return motor;
	}

	public static SpeedController buildMotorGroup(MOTOR_TYPE type, int[] motorChannels, boolean reverseProperty) {
		ArrayList<SpeedController> motorList = new ArrayList<SpeedController>();
		for (int motorChannel : motorChannels) {
			motorList.add(buildMotor(type, motorChannel, reverseProperty));
		}
		SpeedController first = motorList.get(0);
		motorList.remove(0);
		return new SpeedControllerGroup(first, (SpeedController[]) motorList.toArray());
	}
	
	public static Encoder buildEncoder(int channelA, int channelB){
		try{
			return new Encoder(channelA, channelB);
		}catch ( Exception ex ){
			System.err.println(ex.getMessage());
		}
		
		return null;
	}
	
	public static DigitalInput buildDigitalIn(int channel){
		
		try {
			return new DigitalInput(channel);
		} catch ( Exception ex ){
			System.err.println(ex.getMessage());
		}
		
		return null;
	}
	
	public static AnalogInput buildAnalogIn(int channel){
		
		try {
			return new AnalogInput(channel);
		} catch ( Exception ex ) {
			System.err.println(ex.getMessage());
		}
		
		return null;
	}
	
}
