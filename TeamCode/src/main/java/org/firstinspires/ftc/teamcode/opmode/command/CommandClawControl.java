package org.firstinspires.ftc.teamcode.opmode.command;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.Claw;

public class CommandClawControl extends CommandBase {
    private final Claw claw;
    private final double pos;
    public CommandClawControl(Claw claw,double pos){
        this.claw = claw;
        this.pos = pos;
    }
    @Override
    public void initialize(){
        claw.setClawAngle(pos);
        claw.setClawRotationAngle(pos,pos);
        claw.setClawVerticalAngle(pos);
    }
    @Override
    public boolean isFinished(){
        return true;
    }

}
