package org.firstinspires.ftc.teamcode.opmode.tester;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "tutorial")

public class tutorial extends OpMode{
    DcMotor leftSlider;
    DcMotor rightSlider;
    double ticks = 751.8;
    double newTarget;
    @Override
    public void init(){
        leftSlider = hardwareMap.get(DcMotor.class,"LeftSlider");
        rightSlider = hardwareMap.get(DcMotor.class,"RightSlider");
        telemetry.addData("Hardware","Initialized");
        leftSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    @Override
    public void loop(){
        if(gamepad2.a){
            encoder(2);
        }
        telemetry.addData("sliderPos:",SliderCurrentPosition());
        telemetry.update();
    }
    public int SliderCurrentPosition(){
        leftSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        return leftSlider.getCurrentPosition();
    }
    public void encoder(int turnage){
        leftSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        newTarget = ticks*turnage;
        leftSlider.setTargetPosition((int)newTarget);
        leftSlider.setPower(0.5);
        leftSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
