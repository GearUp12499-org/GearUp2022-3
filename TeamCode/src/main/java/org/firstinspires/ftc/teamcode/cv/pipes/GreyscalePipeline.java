package org.firstinspires.ftc.teamcode.cv.pipes;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import static org.opencv.core.CvType.CV_32FC1;

public class GreyscalePipeline extends OpenCvPipeline {
    @Override
    public Mat processFrame(Mat input) {
        Mat tgt = new Mat(input.rows(), input.cols(), CV_32FC1);
        Imgproc.cvtColor(input, tgt, Imgproc.COLOR_RGBA2GRAY);
        return tgt;
    }
}
