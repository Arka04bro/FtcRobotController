package org.firstinspires.ftc.teamcode.opmode.command;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.SubsystemCollection;

public class CommandDriveTrainBrake extends CommandBase {
    private final SubsystemCollection sys;
    private final boolean toggle;

    public CommandDriveTrainBrake(boolean toggle) {
        sys = SubsystemCollection.getInstance(null);
        this.toggle = toggle;
    }

    public void initialize() {
        sys.driveTrain.brake(toggle);
    }

    public boolean isFinished() {
        return true;
    }
}
