package org.firstinspires.ftc.teamcode.opmode.command;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.BooleanSupplier;

public class CommandRunContinuous extends CommandBase {
    private final BooleanSupplier lambda;
    private boolean returned;

    public CommandRunContinuous(BooleanSupplier lambda) {
        this.lambda = lambda;
    }

    public void execute() {
        returned = lambda.getAsBoolean();
    }

    public boolean isFinished() {
        return returned;
    }
}