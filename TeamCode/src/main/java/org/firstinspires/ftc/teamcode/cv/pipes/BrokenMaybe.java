package org.firstinspires.ftc.teamcode.cv.pipes;

import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

/**
 * For Pandamonium
 */
public class BrokenMaybe extends OpenCvPipeline {
    @Override
    public Mat processFrame(Mat input) {
        Mat result = new Mat(input.width() / 2, input.height() / 2, input.type());
        Imgproc.resize(input, result, new Size(input.width() / 2, input.height() / 2));
        return result;
    }
}
