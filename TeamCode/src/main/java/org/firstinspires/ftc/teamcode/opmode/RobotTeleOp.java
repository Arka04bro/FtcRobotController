package org.firstinspires.ftc.teamcode.opmode;


import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmode.command.CommandDriveTrainBrake;
import org.firstinspires.ftc.teamcode.opmode.command.CommandRunContinuous;
import org.firstinspires.ftc.teamcode.subsystem.SubsystemCollection;

/** Основной "Tele-Op" будет использоваться во время соревновательных матчей и тренировок по вождению. */
@TeleOp(name = "Robot TeleOp")
public class RobotTeleOp extends CommandOpMode {
    private SubsystemCollection sys;
    private GamepadEx driver1Gamepad; // driver2Gamepad

    @Override
    public void initialize() {
        SubsystemCollection.deinit();
        sys = SubsystemCollection.getInstance(hardwareMap);

        driver1Gamepad = new GamepadEx(gamepad1);
//        driver2Gamepad = new GamepadEx(gamepad2);


        schedule(new CommandRunContinuous(() -> {
            updateDriver1Controls();
            // updateDriver2Controls();
            // TODO: add telemetry
            return false; // Никогда не закончиться
        }));

        bindDriver1Buttons();
        // bindDriver2Buttons();
    }

    private void updateDriver1Controls() {
        double driveRotationMultiplier = 1.0, driveSpeedMultiplier = 1.0;

        if (driver1Gamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
            driveRotationMultiplier = 0.75;
        } else if (driver1Gamepad.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
            driveRotationMultiplier = 0.5;
        }

        if (driver1Gamepad.getButton(GamepadKeys.Button.DPAD_UP)) {
            driveSpeedMultiplier = 0.75;
        } else if (driver1Gamepad.getButton(GamepadKeys.Button.DPAD_RIGHT)) {
            driveSpeedMultiplier = 0.5;
        } else if (driver1Gamepad.getButton(GamepadKeys.Button.DPAD_DOWN)) {
            driveSpeedMultiplier = 0.25;
        }

        if (driver1Gamepad.getButton(GamepadKeys.Button.DPAD_LEFT)) {
            driveSpeedMultiplier = 0.65;
            driveRotationMultiplier = 0.65;
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
}
