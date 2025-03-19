package org.firstinspires.ftc.teamcode.opmode.command;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import com.arcrobotics.ftclib.command.CommandBase;

public class CommandMoveSliders extends CommandBase {
    private final Intake intake;
    private final int targetPosition;
    private boolean finish = false;

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
    public void execute() {
        if (intake.isAtTargetPosition()) {
            finish = true;
        }
    }

    @Override
    public boolean isFinished() {
        return finish;
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopSliders();
    }
}
