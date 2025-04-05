
package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

    private GamepadEx driver1Gamepad, driver2Gamepad;

    private double driveSpeedMultiplier = Constants.DriveTrain.MAX_SPEED_MULTIPLIER;

    @Override
    public void initialize() {
        SubsystemCollection.deInit();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        sys = SubsystemCollection.getInstance(hardwareMap);

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
     * Управление скоростью движения робота:
     * - Стрелка вверх    — Устанавливает минимальное ограничение скорости.
     * - Стрелка вправо   — Устанавливает среднее ограничение скорости.
     * - Стрелка вниз     — Устанавливает максимальное ограничение скорости.
     * <p>
     * Дополнительные функции:
     * - A (удержание) — Делает движение робота резким, предотвращая движение по инерции после отпускания стиков.
     */
    private void bindDriver1Buttons() {
        driver1Gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(() -> driveSpeedMultiplier = Constants.DriveTrain.MAX_SPEED_MULTIPLIER);
        driver1Gamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(() -> driveSpeedMultiplier = Constants.DriveTrain.MID_SPEED_MULTIPLIER);
        driver1Gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(() -> driveSpeedMultiplier = Constants.DriveTrain.MIN_SPEED_MULTIPLIER);

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
        sys.driveTrain.mecanumDrive.driveRobotCentric(
                driver1Gamepad.getLeftX() * driveSpeedMultiplier,
                driver1Gamepad.getLeftY() * driveSpeedMultiplier,
                driver1Gamepad.getRightX() * driveSpeedMultiplier,
                true
        );
    }

    private void updateDriver2Controls() {
    }

    private void updateTelemetry() {
        telemetry.addLine("Robot Info");

        telemetry.addLine("Drivetrain info");
        telemetry.addData("Speed Multiplier", driveSpeedMultiplier);

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

