package org.firstinspires.ftc.teamcode.opmode.command;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.Claw;

public class CommandClawTake extends CommandBase {
    private final Claw claw;
    private final double output;
    public CommandClawTake(Claw claw,double output){
        this.claw = claw;
        this.output = output;
        addRequirements(claw);
    }
    @Override
    public void initialize(){
        claw.ClawControl(output);
    }
    @Override
    public boolean isFinished(){
        return true;
    }
}
