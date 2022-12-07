package org.firstinspires.ftc.teamcode.nav;

import static org.firstinspires.ftc.teamcode.SharedHardware.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.util.NotImplemented;

import java.util.List;

@Autonomous(name = "Stupid Auto", group = "!!!!!!!!")
public class StupidAuto extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";

    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };
    private static final String VUFORIA_KEY =
            "AYIXAdH/////AAABmXqT3XGDC0CZuA/o5ujewj9a+vrg5ZIXesHnI+83Rf32bNONF8DrPLULld4QxCU21gqKHbcFX0drNV76IXwjdi6m3Aab1qN/Lq8z3f/jsnLTQJ0WV1qgET8C0vnU89rNWZDr3gpFsI/okWlJHmb5fD+mNQ2Rvzxgka0lxKJCfWJo2EgjXQTrE/jVUA7a+MkCl5E0MYlh76CB7uKu9slh8jEa/A/cErML1UQ5VYpdk9RtCfx+qtQqoERBFCo+WsJk/kNPr33EkytMjwlpouLMnmoTFXG4nLpAbSqgkOjqbxm8dmJhy02cTViwWBxL8+4wi4nV5w/Z/dsjrzekWNQ09LYsqHqUsnpntuNqmcK6zJXL";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    int target = 0;
    boolean lastA = false;
    @Override
    public void runOpMode() throws InterruptedException {
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.setZoom(1.0, 16.0 / 9.0);
        }

//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        prepareHardware(hardwareMap);
        EncoderNavigation nav = new EncoderNavigation(
                frontLeft, frontRight, rearLeft, rearRight,
                encoderLeft, encoderRight, encoderRear
        );
        Paths path = new Paths(nav);
        while (opModeInInit()) {


            /*
            telemetry.addData("Current Target", target + 1);
            telemetry.addLine("SMASH gp1 a button to change");
            telemetry.update();
            if (gamepad1.a && !lastA) {
                target ++;
                target %= 3;
            }
            lastA = gamepad1.a;
            */
        }

        telemetry.addData("Selected target: ", target);
        telemetry.update();
        if (tfod != null){
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if(updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                int i = 0;
                for(Recognition recognition : updatedRecognitions){
                    double col = (recognition.getLeft() + recognition.getRight()) / 2 ;
                    double row = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                    double width  = Math.abs(recognition.getRight() - recognition.getLeft()) ;
                    double height = Math.abs(recognition.getTop()  - recognition.getBottom()) ;

                    telemetry.addData(""," ");
                    telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
                    telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
                    telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);

                    if (recognition.getLabel().equals("1 Bolt"))
                        target = 1;
                    else if (recognition.getLabel().equals("2 Bulb"))
                        target = 2;
                    else if (recognition.getLabel().equals("3 Panel"))
                        target = 3;
                }
                telemetry.update();
            }
        }

        //if (target != 0)
            //break;
        nav.moveForward(2);
      /*  switch (target) {
            case 0:
                path.zone1();
                break;
            case 1:
                path.zone2();
                break;
            case 2:
                path.zone3();
                break;
            default:
                throw NotImplemented.i;
        }

       */
        /*while (opModeIsActive()) {
            nav.asyncLoop();
            nav.dumpTelemetry(telemetry);
            telemetry.update();
        }*/
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }
}
