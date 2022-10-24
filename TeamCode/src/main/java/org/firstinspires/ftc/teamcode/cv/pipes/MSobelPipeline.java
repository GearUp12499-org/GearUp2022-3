package org.firstinspires.ftc.teamcode.cv.pipes;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

import static org.opencv.core.CvType.CV_32FC1;

public class MSobelPipeline extends OpenCvPipeline {
    @Override
    public Mat processFrame(Mat input) {
        Mat tgt = new Mat(input.rows(), input.cols(), CV_32FC1);
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
        // AND the red and green channels
        Core.divide(255, r, r);
        Core.divide(255, g, g);
        Core.divide(255, b, b);
        Core.multiply(r, g, r);


        return tgt;
    }
}
