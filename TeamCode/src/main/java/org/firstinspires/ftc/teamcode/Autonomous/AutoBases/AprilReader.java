package org.firstinspires.ftc.teamcode.Autonomous.AutoBases;

import android.content.res.AssetManager;
import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.io.IOException;
import java.io.InputStream;
import java.util.Properties;

public class AprilReader {
    private AprilTagProcessor tagProcessor;

    public AprilReader(HardwareMap hardwareMap) {
        Position cameraPosition = new Position(DistanceUnit.INCH, -7, 3.5, 13.5, 0);
        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 90, -90, 0, 0);
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();
        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .build();

    }

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

//    public double[] convertToRR(int id, double x, double y, double yaw) {
//        double[] coordinates = getAprilTagCoordinates(id);
//        double tagX = coordinates[0];
//        double tagY = coordinates[1];
//
//        if (id == 11 || id == 16) {
//            double cameraPosX = tagX + y + 7;
//            double cameraPosY = tagY - x - 3.5;
//            return new double[]{cameraPosX, cameraPosY, yaw};
//        } else if (id == 12) {
//            double cameraPosX = -x - 3.5;
//            double cameraPosY = tagY - y - 7;
//            return new double[]{cameraPosX, cameraPosY, yaw};
//        } else if (id == 13 || id == 14) {
//            double cameraPosX = tagX - y - 7;
//            double cameraPosY = tagY + x + 3.5;
//            return new double[]{cameraPosX, cameraPosY, yaw};
//        } else if (id == 15) {
//            double cameraPosX = x + 3.5;
//            double cameraPosY = tagY + y + 7;
//            return new double[]{cameraPosX, cameraPosY, yaw};
//        }
//        return null;
//    }

    public double[] read() {
        if (!tagProcessor.getDetections().isEmpty()) {
            AprilTagDetection tag = tagProcessor.getDetections().get(0);
            Position pos = tag.robotPose.getPosition();
            YawPitchRollAngles orientation = tag.robotPose.getOrientation();
            return new double[]{pos.x, pos.y, orientation.getYaw()-Math.toRadians(90), tag.id};
        }
        return new double[]{};
    }
}
