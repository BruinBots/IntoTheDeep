package org.firstinspires.ftc.teamcode.EOCVPipelines;

import android.graphics.Color;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class SampleDetectorPipeline extends OpenCvPipeline {

    // Define HSV color ranges for detection
    private static final Scalar LOWER_BLUE = new Scalar(16, 0, 155);
    private static final Scalar UPPER_BLUE = new Scalar(255, 127, 255);

    private static final Scalar LOWER_RED = new Scalar(32, 176, 0);
    private static final Scalar UPPER_RED = new Scalar(255, 255, 132);

    private static final Scalar LOWER_YELLOW = new Scalar(32, 128, 0);
    private static final Scalar UPPER_YELLOW = new Scalar(255, 170, 120);

    private static final int MIN_AREA = 500;
    private static final int MAX_AREA = 20000;

    @Override
    public Mat processFrame(Mat input) {
        Mat colored = new Mat();
        Imgproc.cvtColor(input, colored, Imgproc.COLOR_RGB2YCrCb);

        // Process for each color
        detectColor(colored, input, LOWER_BLUE, UPPER_BLUE, "Blue");
        detectColor(colored, input, LOWER_RED, UPPER_RED, "Red");
        detectColor(colored, input, LOWER_YELLOW, UPPER_YELLOW, "Yellow");

        // Return the modified input to display it on the FTC dashboard
        return input;
    }

    private void detectColor(Mat hsvFrame, Mat displayFrame, Scalar lowerBound, Scalar upperBound, String colorName) {
        // Create mask for color range
        Mat mask = new Mat();
        Core.inRange(hsvFrame, lowerBound, upperBound, mask);

        // Find contours on the mask
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Filter and display contours
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area >= MIN_AREA && area <= MAX_AREA) {
                // Draw bounding rectangle around contour
                Rect boundingRect = Imgproc.boundingRect(contour);
                Imgproc.rectangle(displayFrame, boundingRect.tl(), boundingRect.br(), new Scalar(0, 255, 0), 2);

                // Display color and area information
                Point center = new Point(
                        boundingRect.x + boundingRect.width / 2,
                        boundingRect.y + boundingRect.height / 2
                );
                Imgproc.putText(displayFrame, String.format("%s: %.0f", colorName, area),
                        center, Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255), 1);
            }
        }
    }
}
