package frc.lib14;

import edu.wpi.first.wpilibj.AnalogInput;

public class MaxBotixRangeFinder extends AnalogInput {

	public MaxBotixRangeFinder(int channel) {
		super(channel);
	}

	public double getDistanceInches() {
		double voltage = getVoltage();
		double distance = UtilityMethods.round((voltage * 100) / 2.54, 10);
		return distance;
	}

}
