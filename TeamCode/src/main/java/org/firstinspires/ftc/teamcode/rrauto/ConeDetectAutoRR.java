package org.firstinspires.ftc.teamcode.rrauto;

import static org.firstinspires.ftc.teamcode.SharedHardware.encoderLeft;
import static org.firstinspires.ftc.teamcode.SharedHardware.encoderRear;
import static org.firstinspires.ftc.teamcode.SharedHardware.encoderRight;
import static org.firstinspires.ftc.teamcode.SharedHardware.frontLeft;
import static org.firstinspires.ftc.teamcode.SharedHardware.frontRight;
import static org.firstinspires.ftc.teamcode.SharedHardware.prepareHardware;
import static org.firstinspires.ftc.teamcode.SharedHardware.rearLeft;
import static org.firstinspires.ftc.teamcode.SharedHardware.rearRight;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Lift;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.nav.EncoderNavigation;
import org.firstinspires.ftc.teamcode.nav.Paths;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.List;


@Autonomous(name = "Auto (+Cone) (+RR)", group = "!!!!!!!!")
public class ConeDetectAutoRR extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";


    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };
    private static final String VUFORIA_KEY =
            "AYIXAdH/////AAABmXqT3XGDC0CZuA/o5ujewj9a+vrg5ZIXesHnI+83Rf32bNONF8DrPLULld4QxCU21gqKHbcFX0drNV76IXwjdi6m3Aab1qN/Lq8z3f/jsnLTQJ0WV1qgET8C0vnU89rNWZDr3gpFsI/okWlJHmb5fD+mNQ2Rvzxgka0lxKJCfWJo2EgjXQTrE/jVUA7a+MkCl5E0MYlh76CB7uKu9slh8jEa/A/cErML1UQ5VYpdk9RtCfx+qtQqoERBFCo+WsJk/kNPr33EkytMjwlpouLMnmoTFXG4nLpAbSqgkOjqbxm8dmJhy02cTViwWBxL8+4wi4nV5w/Z/dsjrzekWNQ09LYsqHqUsnpntuNqmcK6zJXL";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    //@Override
    int bestMatch = 0;

    Lift l;

    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        l = new Lift(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        prepareHardware(hardwareMap);
        EncoderNavigation nav = new EncoderNavigation(
                frontLeft, frontRight, rearLeft, rearRight, encoderLeft, encoderRight, encoderRear
        );
        Paths path = new Paths(nav);
        /*
          Activate TensorFlow Object Detection before we wait for the start command.
          Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         */
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.0, 16.0 / 9.0);
        }

        l.openClaw();

        /* Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();
        l.closeClaw();
        sleep(500);
        l.moveVertical(00); // lift claw so cone does not drag ground

        /*
        Map<Integer, Integer> hist = new HashMap<Integer, Integer>() {
            {
                put(1, 0);
                put(2, 0);
                put(3, 0);
            }
        };
        */

        float max_obs_conf = 0;

        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.milliseconds() < 5000) {
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    for (Recognition recognition : updatedRecognitions) {
                        /*
                        double col = (recognition.getLeft() + recognition.getRight()) / 2;
                        double row = (recognition.getTop() + recognition.getBottom()) / 2;
                        double width = Math.abs(recognition.getRight() - recognition.getLeft());
                        double height = Math.abs(recognition.getTop() - recognition.getBottom());

                        telemetry.addData("", " ");
                        telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                        telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
                        telemetry.addData("- Size (Width/Height)", "%.0f / %.0f", width, height);
                        */
                        if (recognition.getConfidence() > max_obs_conf) {
                            if (recognition.getLabel().equals("1 Bolt")) {
                                bestMatch = 1;
                            } else if (recognition.getLabel().equals("2 Bulb")) {
                                bestMatch = 2;
                            } else if (recognition.getLabel().equals("3 Panel")) {
                                bestMatch = 3;
                            }
                            max_obs_conf = recognition.getConfidence();
                        }


                        telemetry.addData("Target: %i", bestMatch);
                    }
                    telemetry.update();
                }
            }
        }

        nav.moveForward(1);

        TrajectorySequence seq;

        switch (bestMatch) {
            case 1:
                seq = RRPaths.zone1(drive);
                break;
            case 3:
                seq = RRPaths.zone3(drive);
                break;
            case 2:
            default:
                seq = RRPaths.zone2(drive);
                break;
        }
        drive.followTrajectorySequenceAsync(seq);

        while (opModeIsActive()) {
            drive.update();  // run run run run run
            telemetry.addData("target detected", bestMatch);

            telemetry.update();
        }
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
        tfodParameters.minResultConfidence = 0.7f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }
}