package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.lib.Constants;
import org.firstinspires.ftc.teamcode.lib.vision.SampleDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class Vision extends SubsystemBase {
    public OpenCvCamera webcam;

    public Vision(HardwareMap hardwareMap) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()
        );
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);

        webcam.openCameraDeviceAsync(
                new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() {
                        webcam.setPipeline(new SampleDetectionPipeline());
                        webcam.startStreaming(Constants.Vision.CAMERA_WIDTH, Constants.Vision.CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);

                        FtcDashboard.getInstance().startCameraStream(webcam, 30);
                    }

                    @Override
                    public void onError(int errorCode) {
                    }
                }
        );
    }
}
