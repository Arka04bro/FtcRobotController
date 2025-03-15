package org.firstinspires.ftc.teamcode.opmode.command;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.SubsystemCollection;

public class CommandMoveSliders extends CommandBase {
    private final SubsystemCollection sys;
    private final int targetPosition;

    public CommandMoveSliders(int targetPosition) {
        sys = SubsystemCollection.getInstance(null);

        this.targetPosition = targetPosition;
    }

    @Override
    public void initialize() {
        sys.intake.setSliderPosition(targetPosition);
    }

    @Override
    public boolean isFinished() {
        return sys.intake.isAtTargetPosition();
    }

    @Override
    public void end(boolean interrupted) {
        sys.intake.stopSliders();
    }
}
