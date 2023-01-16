package org.firstinspires.ftc.teamcode.cv.pipes;

import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class SharpEdgesPipeline extends OpenCvPipeline {
    private int THRESHOLD = 180;

    private static class Pair<A, B> {
        public final A a;
        public final B b;

        public Pair(A a, B b) {
            this.a = a;
            this.b = b;
        }
    }

    Pair<Integer, Integer> topLeftCorner;
    Pair<Integer, Integer> bottomRightCorner;
    int bestSize = 0;

    public void testAreaSize(Mat input, int sY, int sX) {
        List<Pair<Integer, Integer>> frontier = new ArrayList<>();
        List<Pair<Integer, Integer>> dead = new ArrayList<>();
        if (input.get(sY, sX)[0] == 0) return;
        frontier.add(new Pair<>(sY, sX));

        final int MAX_SIZE = 1000;

        int maxX = 0;
        int minX = input.cols();
        int maxY = 0;
        int minY = input.rows();
        int areaSize = 0;
        while (!frontier.isEmpty() && areaSize < MAX_SIZE) {
            Pair<Integer, Integer> current = frontier.remove(0);
            dead.add(current);
            int y = current.a;
            int x = current.b;

            if (y < minY) minY = y;
            if (y > maxY) maxY = y;
            if (x < minX) minX = x;
            if (x > maxX) maxX = x;

            input.put(y, x, 128);
            areaSize++;
            if (y > 4 && input.get(y - 5, x)[0] == 255 && !dead.contains(new Pair<>(y - 5, x))) {
                frontier.add(new Pair<>(y - 5, x));
            }
            if (y < input.rows() - 5 && input.get(y + 5, x)[0] == 255 && !dead.contains(new Pair<>(y + 5, x))) {
                frontier.add(new Pair<>(y + 5, x));
            }
            if (x > 4 && input.get(y, x - 5)[0] == 255 && !dead.contains(new Pair<>(y, x - 5))) {
                frontier.add(new Pair<>(y, x - 5));
            }
            if (x < input.cols() - 5 && input.get(y, x + 5)[0] == 255 && !dead.contains(new Pair<>(y, x + 5))) {
                frontier.add(new Pair<>(y, x + 5));
            }
        }
        if (areaSize > bestSize) {
            bestSize = areaSize;
            topLeftCorner = new Pair<>(minY, minX);
            bottomRightCorner = new Pair<>(maxY, maxX);
        }
    }

    @Override
    public Mat processFrame(Mat input) {
        bestSize = 0;
        Imgproc.resize(input, input, new org.opencv.core.Size(320, 240), 1, 1, Imgproc.INTER_AREA);
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2GRAY);
        int white = 0;
        for (int x = 0; x < input.cols(); x++) {
            for (int y = 0; y < input.rows(); y++) {
                double[] pixel = input.get(y, x);
                if (pixel[0] > THRESHOLD) {
                    white++;
                    pixel[0] = 255;
                } else {
                    pixel[0] = 0;

                }
                input.put(y, x, pixel);
            }
        }
        // hit 100 points across the image in a grid
        // find the largest area

        for (int x = 0; x < input.cols(); x += input.cols() / 10) {
            for (int y = 0; y < input.rows(); y += input.rows() / 10) {
                testAreaSize(input, y, x);
            }
        }

        if (topLeftCorner != null && bottomRightCorner != null) {
            Imgproc.rectangle(input, new org.opencv.core.Point(topLeftCorner.b, topLeftCorner.a), new org.opencv.core.Point(bottomRightCorner.b, bottomRightCorner.a), new Scalar(128, 0, 0), 2);
        }

        StringBuilder status = new StringBuilder();
        status.append("S: ").append(bestSize)
                .append(" TL: ").append(topLeftCorner.b).append(", ").append(topLeftCorner.a)
                .append(" BR: ").append(bottomRightCorner.b).append(", ").append(bottomRightCorner.a);
//        Imgproc.Sobel(input, input, -1, 0, 1);
        Imgproc.putText(input, status.toString(), new org.opencv.core.Point(10, 20), 0, 0.5, new Scalar(255, 255, 255));
        return input;
    }
}
