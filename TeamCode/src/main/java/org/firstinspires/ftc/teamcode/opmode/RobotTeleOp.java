package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.Constants;
import org.firstinspires.ftc.teamcode.opmode.command.CommandDriveTrainBrake;
import org.firstinspires.ftc.teamcode.opmode.command.CommandMoveSliders;
import org.firstinspires.ftc.teamcode.opmode.command.CommandRunContinuous;
import org.firstinspires.ftc.teamcode.subsystem.SubsystemCollection;

/**
 * Основной "Tele-Op" будет использоваться во время соревновательных матчей и тренировок по вождению.
 **/
@TeleOp(name = "Robot TeleOp")
public class RobotTeleOp extends CommandOpMode {
    private SubsystemCollection sys;
    private GamepadEx driver1Gamepad, driver2Gamepad;

    private double driveRotationMultiplier = Constants.DriveTrain.DEFAULT_ROTATION_MULTIPLIER;
    private double driveSpeedMultiplier = Constants.DriveTrain.DEFAULT_SPEED_MULTIPLIER;

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

    private void bindDriver1Buttons() {
        driver1Gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(() -> driveRotationMultiplier = Constants.DriveTrain.MAX_ROTATION_MULTIPLIER);
        driver1Gamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(() -> driveRotationMultiplier = Constants.DriveTrain.MIN_ROTATION_MULTIPLIER);

        driver1Gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(() -> driveSpeedMultiplier = Constants.DriveTrain.MAX_SPEED_MULTIPLIER);
        driver1Gamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(() -> driveSpeedMultiplier = Constants.DriveTrain.MID_SPEED_MULTIPLIER);
        driver1Gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(() -> driveSpeedMultiplier = Constants.DriveTrain.MIN_SPEED_MULTIPLIER);

        driver1Gamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(() -> {
                            driveRotationMultiplier = Constants.DriveTrain.DEFAULT_ROTATION_MULTIPLIER;
                            driveSpeedMultiplier = Constants.DriveTrain.DEFAULT_SPEED_MULTIPLIER;
                        }
                );

        driver1Gamepad.getGamepadButton(GamepadKeys.Button.X)
                .whileActiveContinuous(new CommandDriveTrainBrake(sys.driveTrain, true))
                .whenInactive(new CommandDriveTrainBrake(sys.driveTrain, false));
    }

    private void bindDriver2Buttons() {
        driver2Gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(() -> schedule(new CommandMoveSliders(sys.intake, Constants.Intake.EXTENDED)));
        driver2Gamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(() -> schedule(new CommandMoveSliders(sys.intake, Constants.Intake.SEMI_EXTENDED)));

        driver2Gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(() -> schedule(new CommandMoveSliders(sys.intake, Constants.Intake.RETRACTED)));
    }

    private void updateDriver1Controls() {
        double driveX = driver1Gamepad.getLeftX() * driveSpeedMultiplier;
        double driveY = driver1Gamepad.getLeftY() * driveSpeedMultiplier;

        sys.driveTrain.mecanumDrive.driveRobotCentric(
                driveX,
                driveY,
                driver1Gamepad.getRightX() * driveRotationMultiplier,
                true
        );
    }

    private void updateDriver2Controls() {
    }

    private void updateTelemetry() {
        // TODO: telemetry
        int[] sliderPositions = sys.intake.getSlidersCurrentPosition();
        for (int i = 0; i < sliderPositions.length; i++) {
            telemetry.addData("Slider " + i + " Distance", sliderPositions[i]);
        }
        telemetry.update();
    }

    @Override
    public void reset() {
        super.reset();
        sys.vision.webcam.stopStreaming();
    }
}
