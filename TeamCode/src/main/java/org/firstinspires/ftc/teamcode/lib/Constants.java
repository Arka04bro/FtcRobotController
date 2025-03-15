package org.firstinspires.ftc.teamcode.lib;

import com.acmerobotics.dashboard.config.Config;

@Config
public final class Constants {
    // TODO: Вынести позишины в отдельный метод
    public static final class Intake {
        public static int EXTENDED = -4750;
        public static int SEMI_EXTENDED = -2875;
        public static int RETRACTED = -50;

        public static double POWER = 0.5;
        public static int TOLERANCE = 20;
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
