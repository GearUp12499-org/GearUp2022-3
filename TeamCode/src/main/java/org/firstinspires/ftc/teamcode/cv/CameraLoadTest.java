package org.firstinspires.ftc.teamcode.cv;

import static org.opencv.core.CvType.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.*;

@TeleOp(name = "CameraLoadTest", group = "!!!!!!!!")
public class CameraLoadTest extends LinearOpMode {
    static class PixelCounter extends OpenCvPipeline {
        double[] results;

        @Override
        public Mat processFrame(Mat input) {
            Mat replica = new Mat(input.rows(), input.cols(), CV_8S);
            Imgproc.Canny(input, replica, 100, 200);
            Mat kern = new Mat(3, 3, CV_32F);
            Imgproc.filter2D(input, replica, -1, kern);
            return input;
        }

        public double[] getLatest() {
            return results;
        }
    }

    OpenCvCamera camera;
    private enum CameraState {
        NOT_READY,
        STARTING,
        READY,
        FAILED
    }
    private CameraState cameraState = CameraState.NOT_READY;

    @Override
    public void runOpMode() throws InterruptedException {
        cameraState = CameraState.NOT_READY;
        WebcamName name = hardwareMap.get(WebcamName.class, "Webcam 1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(name);
        cameraState = CameraState.STARTING;
        PixelCounter pipeline = new PixelCounter();
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                cameraState = CameraState.READY;
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
                camera.setPipeline(pipeline);
            }

            @Override
            public void onError(int errorCode) {
                cameraState = CameraState.FAILED;
            }
        });
        waitForStart();
        while (opModeIsActive()) {
            if (cameraState == CameraState.READY) {
                double[] results = pipeline.getLatest();
                telemetry.addData("R", results[0]);
                telemetry.addData("G", results[1]);
                telemetry.addData("B", results[2]);
            } else {
                telemetry.addData("Camera State", cameraState.toString());
            }
            telemetry.update();
            if (cameraState == CameraState.FAILED) {
                telemetry.addLine("Failed to initialize camera");
                telemetry.update();
                sleep(3000);
                break;
            }
        }
    }
}
