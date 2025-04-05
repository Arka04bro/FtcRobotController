package org.firstinspires.ftc.teamcode.opmode.command;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.Claw;

public class CommandMoveClaw extends CommandBase {
    private final Claw claw;
    private final double pos;

    public CommandMoveClaw(Claw claw, double pos) {
        this.claw = claw;
        this.pos = pos;

        addRequirements(claw);
    }

    @Override
    public void initialize() {
        claw.setClawAngle(pos);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
