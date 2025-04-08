package org.firstinspires.ftc.teamcode.opmode.tester;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@Config
@TeleOp
public class IntakePos_Config extends OpMode {
    private final PIDController sliderController = new PIDController(SliderCoefficients.p, SliderCoefficients.i, SliderCoefficients.d);
    private final PIDController angleController = new PIDController(AngleCoefficients.p, AngleCoefficients.i, AngleCoefficients.d);

    private static final double ticks_in_degree = 751.8 / 360.0;
    private static final double ticks_in_mm = 751.8 / 120.0;

    public static class SliderCoefficients {
        private static final double p = 0.008, i = 0, d = 0.0001, f = 0.08;
        public static int target = 0;
    }

    public static class AngleCoefficients {
        public static double p = 0, i = 0, d = 0, f = 0;
        public static int target = 0;
    }

    private static class MotorsAccess {
        public MotorEx leftSlider, rightSlider, leftAngle, rightAngle;

        public MotorsAccess(MotorEx leftSlider, MotorEx rightSlider, MotorEx leftAngle, MotorEx rightAngle) {
            this.leftSlider = leftSlider;
            this.rightSlider = rightSlider;
            this.leftAngle = leftAngle;
            this.rightAngle = rightAngle;
        }
    }

    private MotorsAccess motorsAccess;
    private CRServo clawAngle;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motorsAccess = new MotorsAccess(
                new MotorEx(hardwareMap, "LeftSlider", Motor.GoBILDA.RPM_223),
                new MotorEx(hardwareMap, "RightSlider", Motor.GoBILDA.RPM_223),
                new MotorEx(hardwareMap, "LeftAngle", Motor.GoBILDA.RPM_223),
                new MotorEx(hardwareMap, "RightAngle", Motor.GoBILDA.RPM_223)
        );
        clawAngle = new CRServo(hardwareMap, "ClawAngle");

        motorsAccess.leftSlider.encoder.reset();
        motorsAccess.rightSlider.encoder.reset();
        motorsAccess.leftAngle.encoder.reset();
        motorsAccess.rightAngle.encoder.reset();
        clawAngle.encoder.reset();
    }

    @Override
    public void loop() {
        // NOTE: SliderPIDF
        int sliderPos = motorsAccess.leftSlider.motorEx.getCurrentPosition();
        sliderController.setPID(SliderCoefficients.p, SliderCoefficients.i, SliderCoefficients.d);
        double sliderPIDOutput = sliderController.calculate(sliderPos, SliderCoefficients.target);
        double sliderFF = Math.cos(Math.toRadians(SliderCoefficients.target / ticks_in_mm)) * SliderCoefficients.f;
        double sliderPower = sliderPIDOutput + sliderFF;

        motorsAccess.leftSlider.motorEx.setPower(sliderPower);
        motorsAccess.rightSlider.motorEx.setPower(-sliderPower);

        // NOTE: AnglePIDF
        int anglePos = motorsAccess.leftAngle.motorEx.getCurrentPosition();
        angleController.setPID(AngleCoefficients.p, AngleCoefficients.i, AngleCoefficients.d);
        double anglePIDOutput = angleController.calculate(anglePos, AngleCoefficients.target);
        double angleFF = Math.cos(Math.toRadians(AngleCoefficients.target / ticks_in_degree)) * AngleCoefficients.f;
        double anglePower = anglePIDOutput + angleFF;

        motorsAccess.leftAngle.motorEx.setPower(anglePower);
        motorsAccess.rightAngle.motorEx.setPower(anglePower);

        // NOTE: Telemetry
        telemetry.addLine("Sliders INFO");
        telemetry.addData("LEFT_Slider POS", sliderPos);
        telemetry.addData("RIGHT_Slider POS", motorsAccess.rightSlider.motorEx.getCurrentPosition());
        telemetry.addData("Slider TARGET", SliderCoefficients.target);
        telemetry.addData("Slider ERROR\n", SliderCoefficients.target - sliderPos);

        telemetry.addLine("Angle INFO");
        telemetry.addData("LEFT_Angle POS", anglePos);
        telemetry.addData("RIGHT_Angle POS", motorsAccess.rightAngle.motorEx.getCurrentPosition());
        telemetry.addData("Angle TARGET", AngleCoefficients.target);
        telemetry.addData("Angle ERROR\n", AngleCoefficients.target - anglePos);

        telemetry.addLine("Claw INFO");
        telemetry.addData("Claw POS", clawAngle.getCurrentPosition());
    }
}
