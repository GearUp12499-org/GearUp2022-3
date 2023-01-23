package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.jetbrains.annotations.Contract;

public abstract class LinearCleanupOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        try {
            main();
        } catch (StopOpMode e) {
            // This is expected, so we don't need to do anything.
            RobotLog.i("OpMode stop requested via StopOpMode exception");
        } catch (Exception e) {
            // This is unexpected, so we should log it.
            RobotLog.ee(
                    this.getClass().getName(),
                    NullTools.withDefault(e.getMessage(), "<message was null>"),
                    e
            );
            throw e;
        } finally {
            RobotLog.i("OpMode cleanup: " + this.getClass().getName());
            cleanup();
            RobotLog.i("OpMode cleanup done: " + this.getClass().getName());
        }
    }

    public final void stopMaybe() {
        if (isStopRequested()) {
            throw new StopOpMode();
        }
    }

    public abstract void main() throws InterruptedException;

    /**
     * NEVER sleep or wait in this method. Only use this to stop motors,
     * clean up sensors & cameras, and generally stop the robot from moving.
     */
    public abstract void cleanup();

    /**
     * Stop the op-mode immediately (by throwing StopOpMode) and invoke cleanup.
     * Generally don't use if you want to stop smoothly, try requestOpModeStop() for that.
     * @throws StopOpMode when invoked
     */
    @Contract("-> fail")
    public final void abortRightNowReally() throws StopOpMode {
        requestOpModeStop();
        throw new StopOpMode();
    }


    public final void safeSleep(long millis) {
        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < millis) {
            stopMaybe();
            sleep(1);
        }
    }
}
