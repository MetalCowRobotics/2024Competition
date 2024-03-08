package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import frc.robot.Constants;
import frc.robot.subsystems.ServoMotorSubsystem.ServoMotorSubsystemConfig;

public class Wrist{

    AnalogPotentiometer pot;

    public Wrist(ServoMotorSubsystemConfig config) {
        super(config);
        pot = new AnalogPotentiometer(Constants.ArmConstants.Offsets.WRIST_POT_ANALOG_ID, 3600, -Constants.ArmConstants.Offsets.WRIST_POT_OFFSET);
    }

    @Override
    protected boolean allowPositiveMotion(double angle) {
        return angle >= -10 && angle <= 135;
        // return true;
    }

    @Override
    protected boolean allowNegativeMotion(double angle) {
        return angle >= -10 && angle <= 135;
        // return true;
    }
    
    public void wristUp() {
        super.setTarget(getTargetAngle() + 2.0);
    }

    public void wristDown() {
        super.setTarget(getTargetAngle() - 2.0);
    }
}