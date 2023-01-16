package org.firstinspires.ftc.teamcode.rrauto;

import static org.firstinspires.ftc.teamcode.SharedHardware.prepareHardware;
import static org.firstinspires.ftc.teamcode.SharedHardware.turret;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.DetectPoleV2;
import org.firstinspires.ftc.teamcode.IOControl;
import org.firstinspires.ftc.teamcode.Lift;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Autonomous(name = "TrigCalcAuto", group = "GearUp")
public class TrigCalcAuto extends LinearOpMode {
    int DRIVE_ENCODER_PER_IN = 2500;
    double TURRET_ENCODER_PER_DEG = 750 / 90.0;
    double HORIZONTAL_LIFT_ENCODER_PER_IN = 0;
    Lift lift;

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        prepareHardware(hardwareMap);
        lift = new Lift(hardwareMap);
        IOControl io = new IOControl(hardwareMap);
        DetectPoleV2 detector = new DetectPoleV2(turret, io.distSensorM, lift, true);

        lift.openClaw();

        waitForStart();
        if (isStopRequested()) return;

        lift.closeClaw();
        sleep(500);
        lift.setVerticalTargetManual(850);
        lift.update();

        // drive straight
        Trajectory initialStraight = drive.trajectoryBuilder(new Pose2d())
                .forward(TrigCalculations.initialDrive())
                .build();
        drive.followTrajectory(initialStraight);

        Pose2d currentPos = drive.getPoseEstimate();
        double currentX = currentPos.getX();
        double currentY = currentPos.getY() * -1; // ACCOUNT FOR RR COORD SYSTEM???

        telemetry.addData("x", currentX);
        telemetry.addData("y", currentY);
        telemetry.update();
        sleep(10000);

        // turn turret right to pole using pole detect
        lift.setVerticalTargetManual(1500);
        sleep(1000);
        while (io.distSensorM.getDistance(DistanceUnit.CM) > 250 &&
                Math.abs(turret.getCurrentPosition()) < 700) {
            turret.setPower(-0.35);

            telemetry.addData("distance:", io.distSensorM.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
        turret.setPower(0);

        sleep(1500);
        lift.setHorizontalTargetManual(225);

        sleep(10000);
        return;

        /*
        // turn turret to stack
        double angToStack = TrigCalculations.sumDeltaAngle(currentX, currentY);
        double encoderStackAngle = angToStack * TURRET_ENCODER_PER_DEG;
        while(turret.getCurrentPosition() < encoderStackAngle){
            turret.setPower(0.35);
        }
        turret.setPower(0);
        lift.openClaw();

        double deltaToStack = TrigCalculations.distToStack(currentX, currentY);
        int encoderStackDistance = (int) (deltaToStack * HORIZONTAL_LIFT_ENCODER_PER_IN);
        lift.moveHorizontal(encoderStackDistance);
        lift.closeClaw();
        lift.moveHorizontal(-1 * encoderStackDistance);
        * */

    }
}
