package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.lib.Constants;
import org.firstinspires.ftc.teamcode.opmode.command.CommandDriveTrainBrake;
import org.firstinspires.ftc.teamcode.opmode.command.CommandMoveSliders;
import org.firstinspires.ftc.teamcode.opmode.command.CommandRunClawWheels;
import org.firstinspires.ftc.teamcode.opmode.command.CommandRunContinuous;
import org.firstinspires.ftc.teamcode.subsystem.SubsystemCollection;

/**
 * Основной "Tele-Op" будет использоваться во время соревновательных матчей и тренировок по вождению.
 */
@TeleOp(name = "Robot TeleOp")
public class RobotTeleOp extends CommandOpMode {
    private SubsystemCollection sys;
    private IMU imu;
    YawPitchRollAngles robotOrientation;

    private GamepadEx driver1Gamepad, driver2Gamepad;

    private int currentMultiplierIndex = 2;
    private double driveSpeedMultiplier = Constants.DriveTrain.SPEED_MULTIPLIERS[currentMultiplierIndex];

    @Override
    public void initialize() {
        SubsystemCollection.deInit();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        sys = SubsystemCollection.getInstance(hardwareMap);

        // NOTE: Maybe i need to put it into subsystem idk
        LazyImu lazyImu = new LazyImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));
        imu = lazyImu.get();
        imu.resetYaw();
        robotOrientation = imu.getRobotYawPitchRollAngles();

        driver1Gamepad = new GamepadEx(gamepad1);
        driver2Gamepad = new GamepadEx(gamepad2);

        bindDriver1Buttons();
        bindDriver2Buttons();

        schedule(new CommandRunContinuous(() -> {
            updateDriver1Controls();
            updateDriver2Controls();
            updateTelemetry();
            return false; // Никогда не закончиться
        }));
    }

    /**
     * Бинды для кнопок первого драйвера.
     * <p>
     * Управление скоростью движения робота (три передачи: 0.25, 0.5, 0.85):
     * - Правый бампер    — Увеличивает передачу.
     * - Стрелка вправо   — Уменьшает передачу.
     * <p>
     * Дополнительные функции:
     * - A (удержание) — Делает движение робота резким, предотвращая движение по инерции после отпускания стиков.
     */
    private void bindDriver1Buttons() {
        driver1Gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(() -> {
                    if (currentMultiplierIndex < Constants.DriveTrain.SPEED_MULTIPLIERS.length - 1) {
                        currentMultiplierIndex++;
                    }
                    driveSpeedMultiplier = Constants.DriveTrain.SPEED_MULTIPLIERS[currentMultiplierIndex];
                });

        driver1Gamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(() -> {
                    if (currentMultiplierIndex > 0) {
                        currentMultiplierIndex--;
                    }
                    driveSpeedMultiplier = Constants.DriveTrain.SPEED_MULTIPLIERS[currentMultiplierIndex];
                });

        driver1Gamepad.getGamepadButton(GamepadKeys.Button.A)
                .whileActiveOnce(new CommandDriveTrainBrake(sys.driveTrain, true))
                .whenInactive(new CommandDriveTrainBrake(sys.driveTrain, false));
    }

    /**
     * Бинды для кнопок второго драйвера.
     * <p>
     * Управление позициями лифта (только для LANDTAKE):
     * - L2 — Сворачивает лифт, сохраняя текущую высоту клешни.
     * - R2 — Разворачивает лифт, сохраняя текущую высоту клешни.
     * <p>
     * Управление основными позициями интейка:
     * - A (нажатие) — Режим сбора семпла с поля.
     * - X (нажатие) — Режим сбора семпла с клипсой у борта игрового поля.
     * - Y (нажатие) — Режим установки семпла с клипсой в high chamber.
     * - B (нажатие) — Режим установки семпла в high basket.
     * <p>
     * Управление позициями интейка (только для ENDGAME):
     * - Стрелка вверх  — Режим подтягивания на турник.
     * - Стрелка вниз   — Режим закрепления на турнике.
     * <p>
     * Управление клешнёй:
     * - R1 (удержание) — Непрерывный сбор семпла.
     * - L1 (удержание) — Непрерывный выброс семпла.
     */
    private void bindDriver2Buttons() {
        // TODO: Implement control to L2 and R2 triggers with inverse kinematics

        driver2Gamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(
                        new ParallelCommandGroup(
//                                new CommandMoveSliderRotation(sys.sliderRotation, Constants.Intake.SliderRotation.LAND),
                                new CommandMoveSliders(sys.slider, Constants.Intake.Slider.LAND)
//                                new CommandMoveClaw(sys.claw, Constants.Intake.Claw.LAND)
                        )
                );
        driver2Gamepad.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(
                        new ParallelCommandGroup(
//                                new CommandMoveSliderRotation(sys.sliderRotation, Constants.Intake.SliderRotation.SIDE),
                                new CommandMoveSliders(sys.slider, Constants.Intake.Slider.SIDE)
//                                new CommandMoveClaw(sys.claw, Constants.Intake.Claw.SIDE)
                        )
                );
        driver2Gamepad.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(
                        new ParallelCommandGroup(
//                                new CommandMoveSliderRotation(sys.sliderRotation, Constants.Intake.SliderRotation.HIGH_CHAMBER),
                                new CommandMoveSliders(sys.slider, Constants.Intake.Slider.HIGH_CHAMBER)
//                                new CommandMoveClaw(sys.claw, Constants.Intake.Claw.CHAMBER)
                        )
                );
        driver2Gamepad.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(
                        new ParallelCommandGroup(
//                                new CommandMoveSliderRotation(sys.sliderRotation, Constants.Intake.SliderRotation.HIGH_BASKET),
                                new CommandMoveSliders(sys.slider, Constants.Intake.Slider.HIGH_BASKET)
//                                new CommandMoveClaw(sys.claw, Constants.Intake.Claw.BASKET)
                        )
                );

        driver2Gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(
                        new ParallelCommandGroup(
//                                new CommandMoveSliderRotation(sys.sliderRotation, Constants.Intake.SliderRotation.CHAMBER),
                                new CommandMoveSliders(sys.slider, Constants.Intake.Slider.CHAMBER)
                        )
                );
        driver2Gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(
                        new ParallelCommandGroup(
//                                new CommandMoveSliderRotation(sys.sliderRotation, Constants.Intake.SliderRotation.CHAMBER),
                                new CommandMoveSliders(sys.slider, Constants.Intake.Slider.RETRACTED)
                        )
                );

        driver2Gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whileActiveOnce(new CommandRunClawWheels(sys.claw, 1))
                .whenInactive(new CommandRunClawWheels(sys.claw, 0));
        driver2Gamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whileActiveOnce(new CommandRunClawWheels(sys.claw, -1))
                .whenInactive(new CommandRunClawWheels(sys.claw, 0));
    }

    private void updateDriver1Controls() {
        sys.driveTrain.mecanumDrive.driveFieldCentric(
                driver1Gamepad.getLeftX() * driveSpeedMultiplier,
                driver1Gamepad.getLeftY() * driveSpeedMultiplier,
                driver1Gamepad.getRightX() * driveSpeedMultiplier,
                robotOrientation.getYaw(AngleUnit.DEGREES),
                true
        );
    }

    private void updateDriver2Controls() {
    }

    private void updateTelemetry() {
        telemetry.addLine("Robot Info");

        telemetry.addLine("Drivetrain info");
        telemetry.addData("Speed Multiplier", driveSpeedMultiplier);

        telemetry.addData("Robot Rotation", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("Robot Y", imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES));
        telemetry.addData("Robot X", imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES));

        telemetry.addData("FrontLeft", hardwareMap.voltageSensor.get("FrontLeft").getVoltage());
        telemetry.addData("FrontRight", hardwareMap.voltageSensor.get("FrontRight").getVoltage());
        telemetry.addData("BackLeft", hardwareMap.voltageSensor.get("BackLeft").getVoltage());
        telemetry.addData("BackRight", hardwareMap.voltageSensor.get("BackRight").getVoltage());

        telemetry.addLine("Intake info");
        int[] sliderPositions = sys.slider.getSlidersCurrentPosition();
        for (int i = 0; i < sliderPositions.length; i++) {
            telemetry.addData("Slider " + i + " Pos", sliderPositions[i]);
        }

        telemetry.update();
    }


    @Override
    public void reset() {
        super.reset();
        sys.vision.webcam.stopStreaming();
    }
}
