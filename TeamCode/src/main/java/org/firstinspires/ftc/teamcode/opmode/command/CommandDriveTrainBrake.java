package org.firstinspires.ftc.teamcode.opmode.command;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.DriveTrain;

public class CommandDriveTrainBrake extends CommandBase {
    private final DriveTrain driveTrain;
    private final boolean toggle;

    public CommandDriveTrainBrake(DriveTrain driveTrain, boolean toggle) {
        this.driveTrain = driveTrain;
        this.toggle = toggle;

        addRequirements(driveTrain);
    }

    public void initialize() {
        driveTrain.brake(toggle);
    }

    public boolean isFinished() {
        return true;
    }
}
