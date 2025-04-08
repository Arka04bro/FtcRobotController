package org.firstinspires.ftc.teamcode.lib;

import com.acmerobotics.dashboard.config.Config;

@Config
public final class Constants {
    public static final class Vision {
        public static int CAMERA_WIDTH = 640;
        public static int CAMERA_HEIGHT = 360;

        public static int MIN_CONTOUR_LENGTH = 290;
        public static int MAX_CONTOUR_LENGTH = 675;

        public static double MIN_DISTANCE = 65;
        public static double PIX2INCHES = 0.015;
    }

    // TODO: Test and rewrite Position and PIDF values
    public static final class Intake {
        public static final class Slider {
            public static int HIGH_BASKET = 3500;
            public static int LOW_BASKET = 0;
            public static int HIGH_CHAMBER = 0;
            public static int LOW_CHAMBER = 0;
            public static int SIDE = 0;
            public static int LAND = 0;
            public static int CHAMBER = 0;
            public static int RETRACTED = 0;

            public static double kP = 0.008;
            public static double kI = 0;
            public static double kD = 0.0001;
            public static double kF = 0.08;

            public static double TICKS_IN_MM = 751.8 / 120.0;
        }

        public static final class SliderRotation {
            public static int HIGH_BASKET = 0;
            public static int LOW_BASKET = 0;
            public static int HIGH_CHAMBER = 0;
            public static int LOW_CHAMBER = 0;
            public static int SIDE = 0;
            public static int LAND = 0;
            public static int CHAMBER = 0;
            public static int RETRACTED = 0;

            public static double kP = 0;
            public static double kI = 0;
            public static double kD = 0;
            public static double kF = 0;

            public static double TICKS_IN_DEGREE = 751.8 / 360.0;
        }

        public static final class Claw {
            public static int BASKET = 0;
            public static int CHAMBER = 0;
            public static int SIDE = 0;
            public static int LAND = 0;
            public static int RETRACTED = 0;

            public static int DESIRED_HEIGHT = 0;
        }
    }


    public static final class DriveTrain {
        public static final double[] SPEED_MULTIPLIERS = {0.25, 0.5, 0.85};
    }
}
