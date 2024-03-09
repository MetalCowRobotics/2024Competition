/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lib14;

import edu.wpi.first.wpilibj.Timer;

/**
 * Add your docs here.
 */
public class TimedCommandSet implements MCRCommand {
    private Timer timer = new Timer();
    private boolean firstTime = true;
    private double targetTime = 15;
    private MCRCommand commandSet;

    public TimedCommandSet(MCRCommand command, double timeout){
        targetTime = timeout;
        commandSet = command;
    }
    
    @Override
    public void run() {
        if (firstTime) {
            firstTime = false;
            startTimer();
        }
        commandSet.run();
    }

    @Override
    public boolean isFinished() {
        if (timerUp()) {
            endTimer();
            return true;
        }
        return commandSet.isFinished();
    }
    private void startTimer() {
        timer.reset();
        timer.start();
    }

    private void endTimer() {
	    timer.stop();
    }

    private boolean timerUp() {
        return timer.get() > targetTime;
    }
}
