package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.lib.Constants;

public class Intake extends SubsystemBase {
    private static class MotorsAccess {
        public Motor leftSlider, rightSlider;

        public MotorsAccess(Motor leftSlider, Motor rightSlider) {
            this.leftSlider = leftSlider;
            this.rightSlider = rightSlider;
        }
    }

    private final MotorsAccess motorsAccess;

    public Intake(HardwareMap hardwareMap) {
        motorsAccess = new MotorsAccess(
                new Motor(hardwareMap, "LeftSlider", Motor.GoBILDA.RPM_223),
                new Motor(hardwareMap, "RightSlider", Motor.GoBILDA.RPM_223)
        );
        motorsAccess.leftSlider.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorsAccess.rightSlider.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        motorsAccess.leftSlider.resetEncoder();
        motorsAccess.rightSlider.resetEncoder();

        motorsAccess.leftSlider.setRunMode(Motor.RunMode.PositionControl);
        motorsAccess.rightSlider.setRunMode(Motor.RunMode.PositionControl);

        motorsAccess.leftSlider.setPositionTolerance(Constants.Intake.TOLERANCE);
        motorsAccess.rightSlider.setPositionTolerance(Constants.Intake.TOLERANCE);
    }

    public void setSliderPosition(int targetPosition) {
        motorsAccess.leftSlider.setTargetPosition(targetPosition);
        motorsAccess.rightSlider.setTargetPosition(targetPosition);

        motorsAccess.leftSlider.set(Constants.Intake.POWER);
        motorsAccess.rightSlider.set(Constants.Intake.POWER);
    }

    public int[] getSlidersCurrentPosition() {
        return new int[]{
                motorsAccess.leftSlider.getCurrentPosition(),
                motorsAccess.rightSlider.getCurrentPosition()
        };
    }

    public boolean isAtTargetPosition() {
        return motorsAccess.leftSlider.atTargetPosition() && motorsAccess.rightSlider.atTargetPosition();
    }

    public void stopSliders() {
        motorsAccess.leftSlider.stopMotor();
        motorsAccess.rightSlider.stopMotor();
    }
}
