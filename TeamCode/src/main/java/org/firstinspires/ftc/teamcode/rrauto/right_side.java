package org.firstinspires.ftc.teamcode.rrauto;

import static org.firstinspires.ftc.teamcode.SharedHardware.frontLeft;
import static org.firstinspires.ftc.teamcode.SharedHardware.frontRight;
import static org.firstinspires.ftc.teamcode.SharedHardware.rearLeft;
import static org.firstinspires.ftc.teamcode.SharedHardware.rearRight;
import static org.firstinspires.ftc.teamcode.SharedHardware.runtime;
import static org.firstinspires.ftc.teamcode.SharedHardware.turret;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name = "right_side", group = "Pushbot")
//@Disabled
public class right_side extends rrAutoComp3 {

    // Constructor
    public right_side() {
        position = "right_side";
    }

    @Override
    void main_auto_content(int targetLocation) throws InterruptedException {
        l.verticalLift(2700, this);
        straight(0.6, 54); // 54 function for driving straight

        //resets turret and lift to home position, ready to be used in teleop, strafes to correct parking position based on what april tag position was detected
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (targetLocation == 1) {
            turret.setTargetPosition(0);
            turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turret.setPower(-0.3);
            sleep(800);
            strafe(0.6, -18);
            //sleep(1500);
            while (l.liftVertical1.getCurrentPosition() > 40) {
                stopMaybe();
                l.liftVertical1.setPower(-0.5);
                l.liftVertical2.setPower(-0.5);
            }
            l.liftVertical1.setPower(0);
            l.liftVertical2.setPower(0);
        } else if (targetLocation == 3) {
            turret.setTargetPosition(0);
            turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turret.setPower(-0.3);
            straight(0.6, 60);
            sleep(800);
            strafe(0.6, 17);
            //sleep(1500);
            while (l.liftVertical1.getCurrentPosition() > 40) {
                stopMaybe();
                l.liftVertical1.setPower(-0.5);
                l.liftVertical2.setPower(-0.5);
            }
            l.liftVertical1.setPower(0);
            l.liftVertical2.setPower(0);
        } else {
            turret.setTargetPosition(0);
            turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turret.setPower(-0.3);
            sleep(800);
            runtime.reset();
            while (runtime.seconds() < 0.8) {
                stopMaybe();
                frontLeft.setPower(-0.4);
                frontRight.setPower(-0.4);
                rearLeft.setPower(-0.4);
                rearRight.setPower(-0.4);
            }
            frontLeft.setPower(0);
            frontRight.setPower(0);
            rearLeft.setPower(0);
            rearRight.setPower(0);
            while (l.liftVertical1.getCurrentPosition() > 40) {
                stopMaybe();
                l.liftVertical1.setPower(-0.5);
                l.liftVertical2.setPower(-0.5);
            }
            l.liftVertical1.setPower(0);
            l.liftVertical2.setPower(0);
        }
    }
}
