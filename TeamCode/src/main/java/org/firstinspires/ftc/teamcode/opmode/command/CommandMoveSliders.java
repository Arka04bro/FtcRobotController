package org.firstinspires.ftc.teamcode.opmode.command;

import org.firstinspires.ftc.teamcode.subsystem.Slider;

import com.arcrobotics.ftclib.command.CommandBase;

public class CommandMoveSliders extends CommandBase {
    private final Slider slider;
    private final int targetPosition;

    public CommandMoveSliders(Slider slider, int targetPosition) {
        this.slider = slider;
        this.targetPosition = targetPosition;

        addRequirements(slider);
    }

    @Override
    public void initialize() {
        slider.setSliderPosition(targetPosition);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
