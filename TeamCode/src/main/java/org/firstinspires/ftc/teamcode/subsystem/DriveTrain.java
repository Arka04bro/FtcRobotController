package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
    private static class MotorsAccess {
        public Motor frontLeft, frontRight, backLeft, backRight;

        public MotorsAccess(Motor frontLeft, Motor frontRight, Motor backLeft, Motor backRight) {
            this.frontLeft = frontLeft;
            this.frontRight = frontRight;
            this.backLeft = backLeft;
            this.backRight = backRight;
        }
    }

    private final MotorsAccess motorsAccess;
    public MecanumDrive mecanumDrive;

    public DriveTrain(HardwareMap hardwareMap) {
        motorsAccess = new MotorsAccess(
                new Motor(hardwareMap, "FrontLeft"),
                new Motor(hardwareMap, "FrontRight"),
                new Motor(hardwareMap, "BackLeft"),
                new Motor(hardwareMap, "BackRight")
        );

        mecanumDrive = new MecanumDrive(
                false,
                motorsAccess.frontLeft,
                motorsAccess.frontRight,
                motorsAccess.backLeft,
                motorsAccess.backRight
        );
        motorsAccess.frontLeft.setInverted(true);
    }

    public void brake(boolean toggle) {
        Motor.ZeroPowerBehavior behavior = toggle ? Motor.ZeroPowerBehavior.BRAKE : Motor.ZeroPowerBehavior.FLOAT;

        for (Motor motor : new Motor[]{motorsAccess.frontLeft, motorsAccess.frontRight, motorsAccess.backLeft, motorsAccess.backRight}) {
            motor.setZeroPowerBehavior(behavior);
        }
    }
}
