package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Claw extends SubsystemBase {
    private static final double minAngle = -135;
    private static final double maxAngle = 135;
    
    private static class ServoAccess {
        public ServoEx servoClaw;
        public CRServo servoLeft,servoRight;

        public ServoAccess(ServoEx servoClaw,CRServo servoLeft,CRServo servoRight) {
            this.servoClaw = servoClaw;
            this.servoLeft = servoLeft;
            this.servoRight = servoRight;
        }
    }

    private final ServoAccess servoAccess;

    public Claw(HardwareMap hardwareMap) {
        servoAccess = new ServoAccess(
                new SimpleServo(hardwareMap, "ServoClaw", minAngle, maxAngle, AngleUnit.DEGREES),
                new CRServo(hardwareMap,"servoLeft"),
                new CRServo(hardwareMap,"ServoRight")
        );
        servoAccess.servoRight.setInverted(true);
    }

    public void ClawControl(double output) {
        servoAccess.servoLeft.set(output);
        servoAccess.servoRight.set(output);
    }

    public void setClawAngle(double angle) {
        servoAccess.servoClaw.turnToAngle(angle);
    }
}

