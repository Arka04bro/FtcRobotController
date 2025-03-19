package org.firstinspires.ftc.teamcode.opmode.command;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import com.arcrobotics.ftclib.command.CommandBase;

public class CommandMoveSliders extends CommandBase {
    private final Intake intake;
    private final int targetPosition;

    public CommandMoveSliders(Intake intake, int targetPosition) {
        this.intake = intake;
        this.targetPosition = targetPosition;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setSliderPosition(targetPosition);
    }

    @Override
    public boolean isFinished() {
        return intake.isAtTargetPosition();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopSliders();
    }
}
