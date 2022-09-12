import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.IOControl;

@TeleOp(name="Read sensor", group="!!!!!!!!")
public class ReadSensor extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        IOControl ioControl = new IOControl(hardwareMap);  // connect to the hardware
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addLine("Distance " + ioControl.distSensor.getDistance(DistanceUnit.CM) + "cm"); // read the distance
            telemetry.update();
        }
    }
}
