package org.firstinspires.ftc.teamcode.opmode.tester;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "DriveTrain TEST")
public class DriveTrainTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Motor frontLeft = new Motor(hardwareMap, "FrontLeft");
        Motor frontRight = new Motor(hardwareMap, "FrontRight");
        Motor backLeft = new Motor(hardwareMap, "BackLeft");
        Motor backRight = new Motor(hardwareMap, "BackRight");

        MecanumDrive drive = new MecanumDrive(true, frontLeft, frontRight, backLeft, backRight);

        waitForStart();

        try {
            // Вперёд
            drive.driveRobotCentric(0, 0.5, 0);
            sleep(1500);
            drive.stop();

            // Назад
            drive.driveRobotCentric(0, -0.5, 0);
            sleep(1500);
            drive.stop();

            // Влево
            drive.driveRobotCentric(-0.5, 0, 0);
            sleep(1500);
            drive.stop();

            // Вправо
            drive.driveRobotCentric(0.5, 0, 0);
            sleep(1500);
            drive.stop();

            // Диагональ вперёд-вправо
            drive.driveRobotCentric(0.5, 0.5, 0);
            sleep(1500);
            drive.stop();

            // Диагональ вперёд-влево
            drive.driveRobotCentric(-0.5, 0.5, 0);
            sleep(1500);
            drive.stop();

            // Диагональ назад-вправо
            drive.driveRobotCentric(0.5, -0.5, 0);
            sleep(1500);
            drive.stop();

            // Диагональ назад-влево
            drive.driveRobotCentric(-0.5, -0.5, 0);
            sleep(1500);
            drive.stop();

            // Поворот по часовой
            drive.driveRobotCentric(0, 0, 0.5);
            sleep(1500);
            drive.stop();

            // Поворот против часовой
            drive.driveRobotCentric(0, 0, -0.5);
            sleep(1500);
            drive.stop();
        } finally {
            drive.stop();
        }
    }
}