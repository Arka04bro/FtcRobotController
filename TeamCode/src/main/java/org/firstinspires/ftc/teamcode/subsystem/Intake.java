package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private static class MotorsAccess {
        public Motor leftVerticalSlider, rightVerticalSlider;

        public MotorsAccess(Motor leftVerticalSlider, Motor rightVerticalSlider) {
            this.leftVerticalSlider = leftVerticalSlider;
            this.rightVerticalSlider = rightVerticalSlider;
        }
    }

    private final MotorsAccess motorsAccess;

    public Intake(HardwareMap hardwareMap){
        motorsAccess = new MotorsAccess(
                new Motor(hardwareMap, "LeftVerticalSlider", Motor.GoBILDA.RPM_223),
                new Motor(hardwareMap, "RightVerticalSlider", Motor.GoBILDA.RPM_223)
        );

        motorsAccess.leftVerticalSlider.resetEncoder();
//        motorsAccess.rightVerticalSlider.resetEncoder();
    }

    public double[] getSlidersDistance() {
        return new double[]{
                motorsAccess.leftVerticalSlider.getDistance(),
//                motorsAccess.rightVerticalSlider.getDistance()
        };
    }

    public void setVerticalSliderPosition(int position) {
        motorsAccess.leftVerticalSlider.setTargetPosition(position);
//        motorsAccess.rightVerticalSlider.setTargetPosition(position);

        motorsAccess.leftVerticalSlider.setRunMode(Motor.RunMode.PositionControl);
//        motorsAccess.rightVerticalSlider.setRunMode(Motor.RunMode.PositionControl);

        motorsAccess.leftVerticalSlider.set(0.5);
//        motorsAccess.rightVerticalSlider.set(0.5);
    }

    public int getSliderCurrentPosition() {
        return motorsAccess.leftVerticalSlider.getCurrentPosition();
//                motorsAccess.rightVerticalSlider.atTargetPosition();
    }

    public void stopSliders() {
        motorsAccess.leftVerticalSlider.stopMotor();
//        motorsAccess.rightVerticalSlider.stopMotor();
    }
}
