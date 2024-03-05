/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lib14;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;

/**
 * Add your docs here.
 */
public class FC_JE_0149Encoder {
    // Encoder encoder = new Encoder(0, 1, true, EncodingType.k2X);
    Encoder encoder;

    public FC_JE_0149Encoder(int channelA, int channelB) {
        encoder = new Encoder(channelA, channelB, true, EncodingType.k2X);
    }

    public double getRate() {     
        return encoder.getRate();
    }
    public int getTics (){
        return encoder.get();
    }
    public void reset (){
        encoder.reset();
    }

    public FC_JE_0149Encoder() {
        // encoder.setDistancePerPulse(.03536);
        // .5" shaft rotates complete 1.57 inches in 44.4 pulses
        encoder.setDistancePerPulse(1.57/44.4);
    }

}
