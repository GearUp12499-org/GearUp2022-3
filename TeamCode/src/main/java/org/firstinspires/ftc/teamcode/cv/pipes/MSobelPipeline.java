package org.firstinspires.ftc.teamcode.cv.pipes;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import static org.opencv.core.CvType.CV_32FC1;
import static org.opencv.imgcodecs.Imgcodecs.imwrite;

public class MSobelPipeline extends OpenCvPipeline {
    File saveItHerePls = AppUtil.ROBOT_DATA_DIR;
    public int c = 0;
    public boolean takePicture;

    @Override
    public Mat processFrame(Mat input) {
        if (!takePicture) return input;
        c += 1;
        takePicture = false;
        List<Mat> channels = new ArrayList<>();
        channels.add(new Mat(input.rows(), input.cols(), CV_32FC1));
        channels.add(new Mat(input.rows(), input.cols(), CV_32FC1));
        channels.add(new Mat(input.rows(), input.cols(), CV_32FC1));
        Core.split(input, channels);
        // put the channels into variables RGB
        Mat r = channels.get(0);
        Mat g = channels.get(1);
        Mat b = channels.get(2);
        // apply sobel to each channel
        Imgproc.Sobel(r, r, CV_32FC1, 1, 0);
        Imgproc.Sobel(g, g, CV_32FC1, 1, 0);
        Imgproc.Sobel(b, b, CV_32FC1, 1, 0);
        // scale down
        Core.divide(255, r, r);
        Core.divide(255, g, g);
        Core.divide(255, b, b);
        // b = 1 - b
        Core.multiply(b, new Scalar(-1), b);
        Core.add(b, new Scalar(1), b);
        // multiply the channels together
        Core.multiply(r, g, r);
        Core.multiply(r, b, r);
        imwrite(new File(saveItHerePls, "snapshot"+c+"_magenta_result.png").toString(), r);
        return r;
        /* ending cap */
    }

    public MSobelPipeline() {
        super();
        AppUtil.getInstance().ensureDirectoryExists(saveItHerePls);
    }
}
