package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.HashMap;
import java.util.Map;

//@TeleOp(name= "April_Tag_3d_Post_Estimation_TEST", group = "org/firstinspires/ftc/teamcode/OpMode")
public class AprilTag_TEST {

    private AprilTagProcessor tagProcessor;
    private VisionPortal visionPortal;

    private HardwareMap hardwareMap;
    private FtcDashboard ftcDashboard;

    public AprilTag_TEST(HardwareMap hardwareMap, FtcDashboard dashboard) {
        this.hardwareMap = hardwareMap;
        this.ftcDashboard = dashboard;
    }

    public void init () {

        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .build();

        visionPortal.resumeStreaming();
        visionPortal.resumeLiveView();

        Map<Integer, Double[]> AprilTagID = new HashMap<>();
        Double[] iD20 = new Double[] {5.5, 7.6};
        Double[] AprilTagCoordinateArray2 = new Double[] {3.4, 6.6};

        AprilTagID.put(1, iD20);
    }

    public void tagProcessing() {
        ftcDashboard.startCameraStream(visionPortal, 30);
    }

    public AprilTagDetection tagUpdate() {

        AprilTagDetection tag = null;

        if (!tagProcessor.getDetections().isEmpty()) {
            tag = tagProcessor.getDetections().get(0);
        }

        return tag;
    }

    public double[] getAprilTagIDLibrary() {
        return null;
    }

}
