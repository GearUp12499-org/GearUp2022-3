package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Run To Odometer Test", group="Testing")
public class RunToClickCount extends LinearOpMode {
    // TODO: get actual data.
    public static int CLICKS_PER_SECOND = 89577;  // 10,000 clicks per second at full speed.
    public static double MAX_ACCELERATION = 0.2;  // How much power is allowed to be added per second.
    public static double MAX_POWER = 1;  // How much power is allowed on the motor.

    /**
     * Runs the robot to a given position by providing power values for motors.
     * Don't run this every loop, it might cause problems. Instead, check the ElapsedTime
     * and run it every ~10ms (adjust if needed)
     * @param distance_to_target How many clicks away the robot is from the target.
     * @param current_power The current power of the motor.
     * @param deltaTime The time since the last call to this function.
     * @return The new power of the motor.
     */
    public static double computeNextPower(int distance_to_target, double current_power, double deltaTime) {
        if (distance_to_target <= 0) return 0.0;
        double eta = current_power == 0 ? Double.MAX_VALUE : distance_to_target / (CLICKS_PER_SECOND * current_power);
        double target_power = Math.min(MAX_POWER, MAX_ACCELERATION * eta);
        double resulting_power = current_power;
        if (target_power != current_power) {
            resulting_power += (target_power > current_power ? deltaTime * MAX_ACCELERATION : -deltaTime * MAX_ACCELERATION);
        }
        return resulting_power;
    }

    @Override
    public void runOpMode() {
        // TODO make this
    }
}
