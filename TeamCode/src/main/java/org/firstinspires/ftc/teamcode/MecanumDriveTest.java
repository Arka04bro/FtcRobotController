package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="MecanumDriveTest", group="OpMode")
public class MecanumDriveTest extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront, rightFront, leftRear, rightRear;

    // Метод инициализации тут короче инициализируем моторы
    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        telemetry.addData("Status", "Initialized");
    }

    // Сброс таймера в начале OpMode
    @Override
    public void start() {
        runtime.reset();
    }

    // Это получается основной метод он запускается после инициализации робота
    @Override
    public void loop() {
        /*
        * Определяем основные направления и силы движения.
        *
        * Левый стик тут получается вычисляем угол направления движения в радианах.
        * Дальше определяем силу движения от 0 до 1 типа если стик в центре power = 0, если на краю power = 1
        */
        double theta = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x);
        double power = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        // ну тут просто поворот вокруг оси
        double turn = gamepad1.right_stick_x;

        // Коррекция угла так как у нас на меканум колесах ролики расположены под углом 45 градусов
        double sin = Math.sin(theta - Math.PI / 4);
        double cos = Math.cos(theta - Math.PI / 4);
        // Тут идет нормализация так сказать масштабирование мощностей чтобы ни одно колесо не получило значение > 1.
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        // ну тут просто вычисляем мощности колес и прокидываем в переменные
        double leftFrontPower = (power * cos / max) + turn;
        double rightFrontPower = (power * sin / max) - turn;
        double leftRearPower = (power * sin / max) + turn;
        double rightRearPower = (power * cos / max) - turn;

        /*
        * Тут делаем нормализацию или по другому масштабируем значения чтобы они не выходили за пределы -1 1
        * потому что у нас сумма power и turn может превысить 1.0
        */
        if ((power + Math.abs(turn)) > 1) {
            leftFrontPower /= (power + Math.abs(turn));
            rightFrontPower /= (power + Math.abs(turn));
            leftRearPower /= (power + Math.abs(turn));
            rightRearPower /= (power + Math.abs(turn));
        }

        // Устанавливаем переменные
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftRear.setPower(leftRearPower);
        rightRear.setPower(rightRearPower);

        // Тут просто вывод телеметрии на телефончике
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Left Front", leftFrontPower);
        telemetry.addData("Right Front", rightFrontPower);
        telemetry.addData("Left Rear", leftRearPower);
        telemetry.addData("Right Rear", rightRearPower);
        telemetry.update();
    }

    @Override
    public void stop() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        telemetry.addData("Status", "Stopped");
        telemetry.update();
    }
}