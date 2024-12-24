package frc.lib14;

import edu.wpi.first.wpilibj.Relay;

public class LightRing {

    Relay relay;

    /**
     * Use this to create a relay that can turn a light ring on and off It is set to
     * only go one direction... you could recode this to handle running two lights
     * but the wiring would be a little different than just straight red-black
     * matchups.
     *
     * @param portNum RoboRio Relay Port Number 0-3
     * @see https://wpilib.screenstepslive.com/s/3120/m/7912/l/132406-on-off-control-of-motors-and-other-mechanisms-relays
     */
    public LightRing(int portNum) {
        this.relay = new Relay(portNum, Relay.Direction.kForward); // only allows you to turn on one side/direction
    }

    /*
     * Notes for if anyone ever looks at this again... kOff - Turns both relay
     * outputs off kForward - Sets the relay to forward (M+ @ 12V, M- @ GND)
     * kReverse - Sets the relay to reverse (M+ @ GND, M- @ 12V) KOn - Sets both
     * relay outputs on (M+ @ 12V, M- @ 12V). Note that if the relay direction is
     * set such that only the forward or reverse pins are enabled this method will
     * be equivalent to kForward or kReverse, however it is not recommended to use
     * kOn in this manner as it may lead to confusion if the relay is later changed
     * to use kBothDirections. Using kForward and kReverse is unambiguous regardless
     * of the direction setting.
     */

    /**
     * Turn the light on
     */
    public void on() {
        this.relay.set(Relay.Value.kForward);
    }

    /**
     * Turn the light off
     */
    public void off() {
        this.relay.set(Relay.Value.kOff);
    }

}