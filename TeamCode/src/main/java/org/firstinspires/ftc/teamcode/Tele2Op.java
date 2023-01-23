package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.lib.NullTools;
import org.firstinspires.ftc.teamcode.lib.StopOpMode;

@TeleOp(name = "<wip> Tele2Op", group = "!!!!!!!!")
public class Tele2Op extends LinearOpMode {
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

    public void main() throws InterruptedException {

    }

    /**
     * NEVER sleep or wait in this method. Only use this to stop motors,
     * clean up sensors & cameras, and generally stop the robot from moving.
     */
    public void cleanup() {

    }
}
