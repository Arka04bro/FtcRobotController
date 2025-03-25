package org.firstinspires.ftc.teamcode.opmode.tester.CAMERA;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.opmode.tester.CAMERA.SampleDetectionPipeline;
import org.firstinspires.ftc.teamcode.opmode.tester.CAMERA.SampleDetectionTesting;
import org.firstinspires.ftc.vision.VisionPortal;

public class Camera {
    private double cameraXRobot;
    private double cameraYRobot;
    private double cameraZRobot;
    private double cameraDistance;
    private final double CAMERA_YAW_ROBOT = 90.0;
    private final double PIVOT_OFFSET = 2.55;
    private final double TUBE_LENGTH = 16.75;
    private final double CAMERA_OFFSET = 1.00; //need to check value (also possibly negative)
    private final double CHASSIS_HEIGHT = 5.73;

    private Telemetry telemetry = null;

    private VisionPortal samplePortal;

    private final SampleDetectionProcessor samples;


    public Camera(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        samples = new SampleDetectionProcessor.Builder()
                .createSampleDetector(telemetry)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        builder.setCameraResolution(new Size(1920,1080));

        builder.addProcessor(samples);

        samplePortal = builder.build();

    }

    public void findCameraPos(double shoulderAngle, double extension){
        cameraXRobot = CAMERA_OFFSET;
        cameraYRobot = Math.cos(shoulderAngle) * (TUBE_LENGTH + extension) + PIVOT_OFFSET;
        cameraZRobot = CHASSIS_HEIGHT + ((TUBE_LENGTH+extension) * Math.sin(shoulderAngle));
        cameraDistance = Math.hypot(cameraXRobot, cameraYRobot);
    }

    public void findYellowSample(){
        samples.enableSampleDetection();
        samples.setFilterToYellow();
    }
    public void findRedSample(){
        samples.enableSampleDetection();
        samples.setFilterToRed();
    }
    public void findBlueSample(){
        samples.enableSampleDetection();
        samples.setFilterToBlue();
    }

    public double getTempX(){
        return samples.getFoundSamplePositionX();
    }
    public double getTempY(){
        return  samples.getFoundSamplePositionY();
    }
    public double getTempYaw(){
        return  samples.getFoundSamplePositionYaw();
    }

    private double getCameraXRobot(){
        return cameraXRobot;
    }
    private double getCameraYRobot() {
        return cameraYRobot;
    }

    private double getCameraZRobot(double shoulderAngle, double extension){
        return cameraZRobot;
    }

    private double getCameraDistance(){
        return cameraDistance;
    }

    private double getCameraYawRobot(){
        return CAMERA_YAW_ROBOT;
    }

    public void startDetection(){
        samples.enableSampleDetection();
    }
    public void stopDetection(){
        samples.disableSampleDetection();
    }

    public boolean didWeFindOne(){
        return samples.isFoundSample();
    }


}
