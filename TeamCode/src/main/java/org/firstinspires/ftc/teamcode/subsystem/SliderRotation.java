package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.lib.Constants;

public class SliderRotation extends SubsystemBase {
    private PIDController controller;
    private int targetPosition = 0;

    private static class MotorsAccess {
        public MotorEx sliderRotateLeft, sliderRotateRight;

        public MotorsAccess(MotorEx sliderRotateLeft, MotorEx sliderRotateRight) {
            this.sliderRotateLeft = sliderRotateLeft;
            this.sliderRotateRight = sliderRotateRight;
        }
    }

    private final MotorsAccess motorsAccess;

    public SliderRotation(HardwareMap hardwareMap) {
        controller = new PIDController(
                Constants.Intake.SliderRotation.kP,
                Constants.Intake.SliderRotation.kI,
                Constants.Intake.SliderRotation.kD
        );
        motorsAccess = new MotorsAccess(
                new MotorEx(hardwareMap, "SliderRotateLeft", Motor.GoBILDA.RPM_223),
                new MotorEx(hardwareMap, "SliderRotateLeft", Motor.GoBILDA.RPM_223)
        );
        motorsAccess.sliderRotateLeft.resetEncoder();
        motorsAccess.sliderRotateRight.resetEncoder();
    }

    public void setTargetPosition(int targetPosition) {
        this.targetPosition = targetPosition;
        controller.reset();
    }

    @Override
    public void periodic() {
        int currentPosition = motorsAccess.sliderRotateLeft.motorEx.getCurrentPosition();
        controller.setPID(Constants.Intake.SliderRotation.kP, Constants.Intake.SliderRotation.kI, Constants.Intake.SliderRotation.kD);
        double pidOutput = controller.calculate(currentPosition, targetPosition);
        double ff = Math.cos(Math.toRadians(targetPosition / Constants.Intake.SliderRotation.TICKS_IN_DEGREE)) * Constants.Intake.SliderRotation.kF;
        double power = pidOutput + ff;

        motorsAccess.sliderRotateLeft.motorEx.setPower(power);
        motorsAccess.sliderRotateRight.motorEx.setPower(power);
    }
}
