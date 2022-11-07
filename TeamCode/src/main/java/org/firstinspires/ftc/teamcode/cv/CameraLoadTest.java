package org.firstinspires.ftc.teamcode.cv;

import static org.opencv.core.CvType.*;
import static org.opencv.imgcodecs.Imgcodecs.imwrite;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.cv.pipes.MSobelPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.*;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "CameraLoadTest", group = "!!!!!!!!")
public class CameraLoadTest extends LinearOpMode {
    static class PixelCounter extends OpenCvPipeline {
        public static final Mat xFilter = new Mat(3, 3, CV_32FC1);
        public static final float[] xFilterDat = {
           -1, 0, 1,
            -2, 0, 2,
            -1, 0, 1
        };

        static {
            xFilter.put(0, 0, xFilterDat);
        }

        double[] results;
        public boolean takePicture;
        public int c = 0;
        File saveItHerePls = AppUtil.ROBOT_DATA_DIR;
//        public static final Mat horizontal = new Mat(3, 3, CV_32FC1, );

        @Override
        public Mat processFrame(Mat input) {
            Mat sobelXB = new Mat(input.rows(), input.cols(), CV_32FC1);
            Mat sobelXG = new Mat(input.rows(), input.cols(), CV_32FC1);
            Mat sobelXR = new Mat(input.rows(), input.cols(), CV_32FC1);
            List<Mat> channels = new ArrayList<>();
            channels.add(new Mat(input.rows(), input.cols(), CV_32FC1));
            channels.add(new Mat(input.rows(), input.cols(), CV_32FC1));
            channels.add(new Mat(input.rows(), input.cols(), CV_32FC1));
            Core.split(input, channels);
            Mat b = channels.get(0);
            Mat g = channels.get(1);
            Mat r = channels.get(2);
            Imgproc.filter2D(b, sobelXB, -1, xFilter);
            Imgproc.filter2D(g, sobelXG, -1, xFilter);
            Imgproc.filter2D(r, sobelXR, -1, xFilter);
            if (!takePicture) return input;
            c += 1;
            takePicture = false;
            imwrite(new File(saveItHerePls, "snapshot"+c+".png").toString(), input);
            imwrite(new File(saveItHerePls, "snapshot"+c+"_sxb.png").toString(), sobelXB);
            imwrite(new File(saveItHerePls, "snapshot"+c+"_sxg.png").toString(), sobelXG);
            imwrite(new File(saveItHerePls, "snapshot"+c+"_sxr.png").toString(), sobelXR);
            return input;
        }

        public double[] getLatest() {
            return results;
        }

        public PixelCounter() {
            AppUtil.getInstance().ensureDirectoryExists(saveItHerePls);
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
        MSobelPipeline pipeline = new MSobelPipeline();
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

        boolean lA = false;
        while (opModeIsActive()) {
            telemetry.addData("Camera State", cameraState.toString());
            if (cameraState == CameraState.READY) {
                telemetry.addData("Taken pictures", pipeline.c);
            }
            telemetry.update();
            if (cameraState == CameraState.FAILED) {
                telemetry.addLine("Failed to initialize camera");
                telemetry.update();
                sleep(3000);
                break;
            }

            if (gamepad1.a && !lA) {
                pipeline.takePicture = true;
            }
            lA = gamepad1.a;
        }
    }
}
