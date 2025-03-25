import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;

public abstract class SampleDetectionProcessor implements VisionProcessor {

    // The position data about where the sample we want to grab is RELATIVE TO CAMERA
    public double foundSamplePositionX = 0;
    public double foundSamplePositionY = 0;
    public double foundSamplePositionYaw = 0;

    public boolean update= false;

    // true only if we found a sample the last time we checked
    public boolean foundSample = false;

    public int colorMode = 2;
    public static class Builder
    {
        SampleDetectionPipeline sampleDetectionPipeline;

        public Builder createSampleDetector(Telemetry telemetry){
            sampleDetectionPipeline = new SampleDetectionPipeline(telemetry);
            return this;
        }

        public SampleDetectionProcessor build()
        {
            return sampleDetectionPipeline;
        }
    }

    public double getFoundSamplePositionX(){
        return foundSamplePositionX;
    }
    public double getFoundSamplePositionY(){
        return foundSamplePositionY;
    }
    public double getFoundSamplePositionYaw(){
        return foundSamplePositionYaw;
    }
    public boolean isFoundSample(){
        return foundSample;
    }

    public void enableSampleDetection(){
        update = true;
    }
    public void disableSampleDetection(){
        update = false;
    }


    public void setFilterToYellow() {
        colorMode = 0;
    }


    public void setFilterToRed(){
        colorMode = 1;
    }

    public void setFilterToBlue(){
        colorMode = 2;
    }

}