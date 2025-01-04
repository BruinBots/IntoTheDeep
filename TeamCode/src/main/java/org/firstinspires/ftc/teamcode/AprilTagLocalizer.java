package org.firstinspires.ftc.teamcode;

import android.content.res.AssetManager;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.io.IOException;
import java.io.InputStream;
import java.util.Properties;

@Autonomous(name = "AprilTag Localization Test")
public class AprilTagLocalizer extends LinearOpMode {

    public double[] getAprilTagCoordinates(int id) {
        AssetManager assetManager = AppUtil.getDefContext().getAssets();
        try (InputStream input = assetManager.open("apriltags.properties")) {
            Properties prop = new Properties();
            prop.load(input);
            String coordinates = prop.getProperty("tag." + id);

            if (coordinates != null) {
                String[] parts = coordinates.split(",");
                if (parts.length == 2) {
                    double x = Double.parseDouble(parts[0]);
                    double y = Double.parseDouble(parts[1]);
                    return new double[]{x, y};
                }
            }

        } catch (IOException ex) {
            ex.printStackTrace();
        }
        return null;
    }

    public double[] convertToRR(int id, double x, double y) {
        double[] coordinates = getAprilTagCoordinates(id);
        double tagX = coordinates[0];
        double tagY = coordinates[1];

        if (id == 11 || id == 16) {
            double cameraPosX = tagX + y + 7;
            double cameraPosY = tagY - x - 3.5;
            return new double[]{cameraPosX, cameraPosY};
        } else if (id == 12) {
            double cameraPosX = -x - 3.5;
            double cameraPosY = tagY - y - 7;
            return new double[]{cameraPosX, cameraPosY};
        } else if (id == 13 || id == 14) {
            double cameraPosX = tagX - y - 7;
            double cameraPosY = tagY + x + 3.5;
            return new double[]{cameraPosX, cameraPosY};
        } else if (id == 15) {
            double cameraPosX = x + 3.5;
            double cameraPosY = tagY + y + 7;
            return new double[]{cameraPosX, cameraPosY};
        }
        return null;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();
        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .build();

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            if (!tagProcessor.getDetections().isEmpty()) {
                AprilTagDetection tag = tagProcessor.getDetections().get(0);
                double[] coords = convertToRR(tag.id, tag.ftcPose.x, tag.ftcPose.y);
                telemetry.addData("Calculated X Position", coords[0]);
                telemetry.addData("Calculated Y Position", coords[1]);
            }
        }
    }
}
