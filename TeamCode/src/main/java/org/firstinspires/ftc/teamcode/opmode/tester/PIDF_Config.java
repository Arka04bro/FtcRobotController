package org.firstinspires.ftc.teamcode.opmode.tester;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@Config
@TeleOp
public class PIDF_Config extends OpMode {
    private PIDController controller;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    public static int target = 0;

    // For 5203 Series Yellow Jacket Planetary Gear Motor 223 RPM
    private static final double ticks_in_degree = 751.8/360.0; // 2,0883333333

    private DcMotorEx motorEx;

    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motorEx = hardwareMap.get(DcMotorEx.class, "Motor_0");
    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int sliderPos = motorEx.getCurrentPosition();
        double pid = controller.calculate(sliderPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid + ff;

        motorEx.setPower(power);

        telemetry.addData("pos ", sliderPos);
        telemetry.addData("target pos ", target);
        telemetry.update();
    }
}
