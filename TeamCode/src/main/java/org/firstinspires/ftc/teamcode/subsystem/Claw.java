package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Claw extends SubsystemBase {
    private static final double minAngle = -135;
    private static final double maxAngle = 135;
    
    private static class ServoAccess {
        public ServoEx servoRight, servoLeft, servoClaw;

        public ServoAccess(ServoEx servoRight, ServoEx servoLeft, ServoEx servoClaw) {
            this.servoRight = servoRight;
            this.servoLeft = servoLeft;
            this.servoClaw = servoClaw;
        }
    }

    private final ServoAccess servoAccess;

    public Claw(HardwareMap hardwareMap) {
        servoAccess = new ServoAccess(
                new SimpleServo(hardwareMap, "ServoRight", minAngle, maxAngle, AngleUnit.DEGREES),
                new SimpleServo(hardwareMap, "ServoLeft", minAngle, maxAngle, AngleUnit.DEGREES),
                new SimpleServo(hardwareMap, "ServoClaw", minAngle, maxAngle, AngleUnit.DEGREES)
        );
    }

    public void setClawVerticalAngle(double angle) {
        servoAccess.servoLeft.turnToAngle(angle);
        servoAccess.servoRight.turnToAngle(angle);
    }

    public void setClawRotationAngle(double leftAngle, double rightAngle) {
        servoAccess.servoLeft.turnToAngle(leftAngle);
        servoAccess.servoRight.turnToAngle(rightAngle);
    }

    public void setClawAngle(double angle) {
        servoAccess.servoClaw.turnToAngle(angle);
    }
}

