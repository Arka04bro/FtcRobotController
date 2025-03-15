package org.firstinspires.ftc.teamcode.opmode;


import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.Constants;
import org.firstinspires.ftc.teamcode.opmode.command.CommandDriveTrainBrake;
import org.firstinspires.ftc.teamcode.opmode.command.CommandRunContinuous;
import org.firstinspires.ftc.teamcode.subsystem.SubsystemCollection;

/** Основной "Tele-Op" будет использоваться во время соревновательных матчей и тренировок по вождению. **/
@TeleOp(name = "Robot TeleOp")
public class RobotTeleOp extends CommandOpMode {
    private SubsystemCollection sys;
    private GamepadEx driver1Gamepad, driver2Gamepad;

    @Override
    public void initialize() {
        SubsystemCollection.deInit();
        sys = SubsystemCollection.getInstance(hardwareMap);

        driver1Gamepad = new GamepadEx(gamepad1);
        driver2Gamepad = new GamepadEx(gamepad2);


        schedule(new CommandRunContinuous(() -> {
            updateDriver1Controls();
             updateDriver2Controls();
            updateTelemetry();
            return false; // Никогда не закончиться
        }));

        bindDriver1Buttons();
//         bindDriver2Buttons();
    }

    private void updateTelemetry() {
        // TODO: telemetry
        for (double distance : sys.intake.getSlidersDistance()) {
            telemetry.addData("Slider Distance: ", distance);
        }
        telemetry.update();
    }

    private void updateDriver1Controls() {
        double driveRotationMultiplier = 1.0, driveSpeedMultiplier = 1.0;

        if (driver1Gamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
            driveRotationMultiplier = Constants.DriveTrain.MAX_ROTATION_MULTIPLIER;
        } else if (driver1Gamepad.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
            driveRotationMultiplier = Constants.DriveTrain.MIN_ROTATION_MULTIPLIER;
        }

        if (driver1Gamepad.getButton(GamepadKeys.Button.DPAD_UP)) {
            driveSpeedMultiplier = Constants.DriveTrain.MAX_SPEED_MULTIPLIER;
        } else if (driver1Gamepad.getButton(GamepadKeys.Button.DPAD_RIGHT)) {
            driveSpeedMultiplier = Constants.DriveTrain.MID_SPEED_MULTIPLIER;
        } else if (driver1Gamepad.getButton(GamepadKeys.Button.DPAD_DOWN)) {
            driveSpeedMultiplier = Constants.DriveTrain.MIN_SPEED_MULTIPLIER;
        }

        if (driver1Gamepad.getButton(GamepadKeys.Button.DPAD_LEFT)) {
            driveSpeedMultiplier = Constants.DriveTrain.DEFAULT_ROTATION_MULTIPLIER;
            driveRotationMultiplier = Constants.DriveTrain.DEFAULT_SPEED_MULTIPLIER;
        }

        double driveX = driver1Gamepad.getLeftX() * driveSpeedMultiplier;
        double driveY = driver1Gamepad.getLeftY() * driveSpeedMultiplier;

        sys.driveTrain.mecanumDrive.driveRobotCentric(
                driveX,
                driveY,
                driver1Gamepad.getRightX() * driveRotationMultiplier,
                true
        );
    }

    private void bindDriver1Buttons() {
        driver1Gamepad.getGamepadButton(GamepadKeys.Button.X)
                .whileActiveContinuous(new CommandDriveTrainBrake(true))
                .whenInactive(new CommandDriveTrainBrake(false));
    }

    private void updateDriver2Controls() {
        int verticalSlider = 0;

        if (driver2Gamepad.getButton(GamepadKeys.Button.DPAD_UP)) {
            verticalSlider = Constants.Intake.EXTENDED;
            sys.intake.setVerticalSliderPosition(verticalSlider);
        } else if (driver2Gamepad.getButton(GamepadKeys.Button.DPAD_RIGHT)) {
            verticalSlider = Constants.Intake.SEMI_EXTENDED;
            sys.intake.setVerticalSliderPosition(verticalSlider);
        } else if (driver2Gamepad.getButton(GamepadKeys.Button.DPAD_DOWN)) {
            verticalSlider = Constants.Intake.RETRACTED;
            sys.intake.setVerticalSliderPosition(verticalSlider);
        }

        // Stop motors when they reach target
        if (sys.intake.getSliderCurrentPosition() <= verticalSlider) {
            sys.intake.stopSliders();
        }
    }
}
