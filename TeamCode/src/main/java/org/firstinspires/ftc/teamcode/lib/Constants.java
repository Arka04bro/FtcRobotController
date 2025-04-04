package org.firstinspires.ftc.teamcode.lib;

import com.acmerobotics.dashboard.config.Config;

@Config
public final class Constants {
    // TODO: Вынести позишины в отдельный метод
    public static final class Vision {
        public static int MIN_CONTOUR_LENGTH = 290;
        public static int MAX_CONTOUR_LENGTH = 675;

        // Note: IDK what is MIN_DISTANCE
        public static double MIN_DISTANCE = 65;
        public static double PIX2INCHES = 0.015;
    }

    public static final class Intake {
        public static int EXTENDED = 6000;
        public static int SEMI_EXTENDED = 3000;
        public static int RETRACTED = 0;

        public static double kP = 0.004;
        public static double kI = 0;
        public static double kD = 0.0001;
        public static double kF = 0.1;
        public static double TICKS_IN_DEGREE = 20272.08 / 120.0;
    }
    public static final class Claw {
        public static double UP = 135;
        public static double DOWN = -135;
        public static double INPUT = 1;
        public static double OUTPUT = -1;
    }

    public static final class DriveTrain {
        public static final double MAX_ROTATION_MULTIPLIER = 0.75;
        public static final double MIN_ROTATION_MULTIPLIER = 0.5;

        public static final double MAX_SPEED_MULTIPLIER = 0.75;
        public static final double MID_SPEED_MULTIPLIER = 0.5;
        public static final double MIN_SPEED_MULTIPLIER = 0.25;

        public static final double DEFAULT_SPEED_MULTIPLIER = 0.65;
        public static final double DEFAULT_ROTATION_MULTIPLIER = 0.65;
    }
}
