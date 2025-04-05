package org.firstinspires.ftc.teamcode.opmode.command;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.SliderRotation;

public class CommandMoveSliderRotation extends CommandBase {
    private final SliderRotation sliderRotation;
    private final int targetPosition;


    public CommandMoveSliderRotation(SliderRotation sliderRotation, int targetPosition) {
        this.sliderRotation = sliderRotation;
        this.targetPosition = targetPosition;

        addRequirements(sliderRotation);
    }

    @Override
    public void initialize() {
        sliderRotation.setTargetPosition(targetPosition);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
