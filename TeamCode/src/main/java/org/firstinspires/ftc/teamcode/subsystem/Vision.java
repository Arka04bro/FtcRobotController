package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class Vision extends SubsystemBase {
    public OpenCvCamera webcam;
    private static final int CAMERA_WIDTH = 640;
    private static final int CAMERA_HEIGHT = 360;

    public Vision(HardwareMap hardwareMap) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()
        );
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        webcam.openCameraDevice();

        webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);

        FtcDashboard.getInstance().startCameraStream(webcam, 30);
    }
}
