package org.firstinspires.ftc.teamcode.opmode.command;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.Claw;

public class CommandRunClawWheels extends CommandBase {
    private final Claw claw;
    private final double power;

    public CommandRunClawWheels(Claw claw, double power) {
        this.claw = claw;
        this.power = power;

        addRequirements(claw);
    }

    @Override
    public void initialize() {
        claw.clawControl(power);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
