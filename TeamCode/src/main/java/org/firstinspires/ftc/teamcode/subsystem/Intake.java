package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.lib.Constants;

public class Intake extends SubsystemBase {
    private PIDController controller;
    private int targetPosition = 0;

    private static class MotorsAccess {
        public MotorEx leftSlider, rightSlider;

        public MotorsAccess(MotorEx leftSlider, MotorEx rightSlider) {
            this.leftSlider = leftSlider;
            this.rightSlider = rightSlider;
        }
    }

    private final MotorsAccess motorsAccess;

    public Intake(HardwareMap hardwareMap) {
        controller = new PIDController(Constants.Intake.kP, Constants.Intake.kI, Constants.Intake.kD);
        motorsAccess = new MotorsAccess(
                new MotorEx(hardwareMap, "LeftSlider", MotorEx.GoBILDA.RPM_223),
                new MotorEx(hardwareMap, "RightSlider", MotorEx.GoBILDA.RPM_223)
        );
        motorsAccess.leftSlider.resetEncoder();
        motorsAccess.rightSlider.resetEncoder();
    }

    public void setSliderPosition(int targetPosition) {
        this.targetPosition = targetPosition;
        controller.reset();
    }

    @Override
    public void periodic() {
        int currentPosition = motorsAccess.leftSlider.motorEx.getCurrentPosition();
        controller.setPID(Constants.Intake.kP, Constants.Intake.kI, Constants.Intake.kD);
        double pidOutput = controller.calculate(currentPosition, targetPosition);
        double ff = Math.cos(Math.toRadians(targetPosition / Constants.Intake.TICKS_IN_DEGREE)) * Constants.Intake.kF;
        double power = pidOutput + ff;

        motorsAccess.leftSlider.motorEx.setPower(power);
        motorsAccess.rightSlider.motorEx.setPower(power);
    }

    public int[] getSlidersCurrentPosition() {
        return new int[]{motorsAccess.leftSlider.motorEx.getCurrentPosition(), motorsAccess.rightSlider.motorEx.getCurrentPosition()};
    }
}
