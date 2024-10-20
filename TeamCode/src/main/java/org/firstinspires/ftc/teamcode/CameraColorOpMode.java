/*
 * Copyright (c) 2024 Phil Malone
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import android.util.Size;

//import com.google.android.gms.drive.query.SortOrder;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.SortOrder;
//import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;

import java.util.List;


@TeleOp(name = "Concept: Vision Color-Locator", group = "Concept")
public class CameraColorOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Build color locators for each color
        ColorBlobLocatorProcessor blueLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(new ColorRange(
                        ColorSpace.HSV,
                        new Scalar(110, 97, 38), // Lower bound for yellow in HSV
                        new Scalar(127, 217, 129)  // Upper bound for yellow in HSV
                ))
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setDrawContours(true)
                .setRoi(ImageRegion.entireFrame())
                .setBlurSize(0)
                .build();

        ColorBlobLocatorProcessor redLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(new ColorRange(
                        ColorSpace.HSV,
                        new Scalar(0, 217, 85), // Lower bound for yellow in HSV
                        new Scalar(6, 255, 167)  // Upper bound for yellow in HSV
                ))
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setDrawContours(true)
                .setRoi(ImageRegion.entireFrame())
                .setBlurSize(0)
                .build();

        ColorBlobLocatorProcessor yellowLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(new ColorRange(
                        ColorSpace.HSV,
                        new Scalar(11, 222, 147), // Lower bound for yellow in HSV
                        new Scalar(27, 255, 255)  // Upper bound for yellow in HSV
                ))
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setDrawContours(true)
                .setRoi(ImageRegion.entireFrame())
                .setBlurSize(0)
                .build();
        // Create the VisionPortal with all color processors
        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(blueLocator)
                .addProcessor(redLocator)
                .addProcessor(yellowLocator)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        telemetry.setMsTransmissionInterval(50);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        while (opModeIsActive() || opModeInInit()) {
            telemetry.addData("preview on/off", "... Camera Stream\n");

            // Get blobs for each color
            List<ColorBlobLocatorProcessor.Blob> blueBlobs = blueLocator.getBlobs();
            List<ColorBlobLocatorProcessor.Blob> redBlobs = redLocator.getBlobs();
            List<ColorBlobLocatorProcessor.Blob> yellowBlobs = yellowLocator.getBlobs();

            // Filter blobs by size for each color
            ColorBlobLocatorProcessor.Util.filterByArea(500, 20000, blueBlobs);
            ColorBlobLocatorProcessor.Util.filterByArea(500, 20000, redBlobs);
            ColorBlobLocatorProcessor.Util.filterByArea(500, 20000, yellowBlobs);

            telemetry.addLine(" Color  Area Density Aspect  Center");

            // Display blobs with color attribute
            for (ColorBlobLocatorProcessor.Blob b : blueBlobs) {
                RotatedRect boxFit = b.getBoxFit();
                telemetry.addLine(String.format("Blue   %5d  %4.2f   %5.2f  (%3d,%3d)",
                        b.getContourArea(), b.getDensity(), b.getAspectRatio(), (int) boxFit.center.x, (int) boxFit.center.y));
            }

            for (ColorBlobLocatorProcessor.Blob b : redBlobs) {
                RotatedRect boxFit = b.getBoxFit();
                telemetry.addLine(String.format("Red    %5d  %4.2f   %5.2f  (%3d,%3d)",
                        b.getContourArea(), b.getDensity(), b.getAspectRatio(), (int) boxFit.center.x, (int) boxFit.center.y));
            }

            for (ColorBlobLocatorProcessor.Blob b : yellowBlobs) {
                RotatedRect boxFit = b.getBoxFit();
                telemetry.addLine(String.format("Yellow %5d  %4.2f   %5.2f  (%3d,%3d)",
                        b.getContourArea(), b.getDensity(), b.getAspectRatio(), (int) boxFit.center.x, (int) boxFit.center.y));
            }

            telemetry.update();
            sleep(50);
        }
    }
}