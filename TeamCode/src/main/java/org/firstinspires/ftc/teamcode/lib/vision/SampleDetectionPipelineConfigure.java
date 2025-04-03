package org.firstinspires.ftc.teamcode.lib.vision;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Config
public class SampleDetectionPipelineConfigure extends OpenCvPipeline {
    // 0 - red, 1 - yellow, 2 - blue
    public static int trackingColorChoice = 0;
    public static boolean showMask = false;

    // Yellow range
    public static double YellowHL = 12;
    public static double YellowHH = 61;
    public static double YellowSL = 119;
    public static double YellowSH = 255;
    public static double YellowVL = 186;
    public static double YellowVH = 255;

    // Red low range
    public static double Red1HL = 159;
    public static double Red1HH = 180;
    public static double Red1SL = 50;
    public static double Red1SH = 255;
    public static double Red1VL = 70;
    public static double Red1VH = 255;

    // Red high range
    public static double Red2HL = 0;
    public static double Red2HH = 9;
    public static double Red2SL = 50;
    public static double Red2SH = 255;
    public static double Red2VL = 70;
    public static double Red2VH = 255;

    // Blue range
    public static double BlueHL = 75;
    public static double BlueHH = 141;
    public static double BlueSL = 108;
    public static double BlueSH = 255;
    public static double BlueVL = 50;
    public static double BlueVH = 255;

    public static int MIN_CONTOUR_LENGTH = 290;
    public static int MAX_CONTOUR_LENGTH = 675;

    public static double MIN_DISTANCE = 65;
    public static double PIX2INCHES = 0.015;

    private final List<Point> foundSamplePositionsPix = new ArrayList<>();
    private final List<Point> foundSamplePositionsInches = new ArrayList<>();
    private final List<Double> foundSampleRotations = new ArrayList<>();

    private static class Sample {
        private final Point centerInInches;
        private final double rotation;

        public Sample( Point centerInInches, double rotation) {
            this.centerInInches = centerInInches;
            this.rotation = rotation;
        }

        public Point getCenterInInches() {
            return centerInInches;
        }

        public double getRotation() {
            return rotation;
        }
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat mask = preprocessFrame(input);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        Sample bestSample = findLargestContour(contours, input);

        for (MatOfPoint contour : contours) {
            Imgproc.drawContours(input, List.of(contour), -1, new Scalar(0, 255, 0), 3);
        }

        if (bestSample != null) {
            Rect boundingRect = Imgproc.boundingRect(new MatOfPoint(bestSample.getCenterInInches()));
            Imgproc.rectangle(input, boundingRect.tl(), boundingRect.br(), new Scalar(0, 255, 0), 3);
            Imgproc.putText(input, "Angle: " + bestSample.getRotation(), new Point(boundingRect.x, boundingRect.y - 10),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, new Scalar(0, 255, 0), 2);
            Imgproc.circle(input, bestSample.getCenterInInches(), 25, new Scalar(0, 255, 0), 9);
        }

        return showMask ? mask : input;
    }

    // TODO: Optimize garbage collector
    private Mat preprocessFrame(Mat frame) {
        Mat hsvFrame = new Mat();
        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

        Mat boundMask = new Mat();

        // 0 - red, 1 - yellow, 2 - blue
        switch (trackingColorChoice) {
            case 1: // yellow
                Scalar yellowLower = new Scalar(YellowHL, YellowSL, YellowVL);
                Scalar yellowUpper = new Scalar(YellowHH, YellowSH, YellowVH);
                Core.inRange(hsvFrame, yellowLower, yellowUpper, boundMask);
                break;
            case 2: // blue
                Scalar blueLower = new Scalar(BlueHL, BlueSL, BlueVL);
                Scalar blueUpper = new Scalar(BlueHH, BlueSH, BlueVH);
                Core.inRange(hsvFrame, blueLower, blueUpper, boundMask);
                break;
            case 0: // red is default
            default:
                Scalar redLower1 = new Scalar(Red1HL, Red1SL, Red1VL);
                Scalar redUpper1 = new Scalar(Red1HH, Red1SH, Red1VH);
                Scalar redLower2 = new Scalar(Red2HL, Red2SL, Red2VL);
                Scalar redUpper2 = new Scalar(Red2HH, Red2SH, Red2VH);
                Mat mask1 = new Mat();
                Mat mask2 = new Mat();
                Core.inRange(hsvFrame, redLower1, redUpper1, mask1);
                Core.inRange(hsvFrame, redLower2, redUpper2, mask2);
                Core.bitwise_or(mask1, mask2, boundMask);
                break;
        }

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.morphologyEx(boundMask, boundMask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(boundMask, boundMask, Imgproc.MORPH_CLOSE, kernel);

        return boundMask;
    }

    /**
     * Приватный метод, который проходит по всем контурам, фильтрует их по периметру,
     * аппроксимирует контур, вычисляет его центр и ориентацию,
     * переводит координаты центра из пикселей в дюймы и выбирает семпл,
     * расстояние центра которого до (0,0) (в инчах) минимально.
     *
     * @param contours  список найденных контуров
     * @param baseImage базовое изображение
     * @return объект Sample с данными лучшего семпла или null, если семплы не найдены
     */
    private Sample findLargestContour(List<MatOfPoint> contours, Mat baseImage) {
        foundSamplePositionsPix.clear();
        foundSamplePositionsInches.clear();
        foundSampleRotations.clear();

        for (MatOfPoint contour : contours) {
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
            double arcLen = Imgproc.arcLength(contour2f, true);
            if (arcLen < MIN_CONTOUR_LENGTH || arcLen > MAX_CONTOUR_LENGTH) {
                continue;
            }

            MatOfPoint2f approxContour2f = new MatOfPoint2f();
            Imgproc.approxPolyDP(contour2f, approxContour2f, arcLen / 22.0, true);
            List<Point> approxPoints = approxContour2f.toList();
            if (approxPoints.isEmpty()) {
                continue;
            }
            MatOfPoint approxContour = new MatOfPoint();
            approxContour.fromList(approxPoints);

            int sumX = 0, sumY = 0;
            List<Point> pts = approxContour.toList();
            for (Point pt : pts) {
                sumX += (int) pt.x;
                sumY += (int) pt.y;
            }
            int avgX = sumX / pts.size();
            int avgY = sumY / pts.size();
            Point centerPix = new Point(avgX, avgY);

            boolean originalPos = true;
            for (Point s : foundSamplePositionsPix) {
                if (distance(centerPix, s) < MIN_DISTANCE) {
                    originalPos = false;
                    break;
                }
            }
            if (!originalPos)
                continue;
            foundSamplePositionsPix.add(centerPix);

            double bestDistance = 0.0;
            Point pos1 = new Point(), pos2 = new Point();
            for (int i = 0; i < pts.size(); i++) {
                Point p1 = pts.get(i);
                Point p2 = pts.get((i + 1) % pts.size());
                double d = distance(p1, p2);
                if (d > bestDistance) {
                    bestDistance = d;
                    pos1 = p1;
                    pos2 = p2;
                }
            }
            Point angleP1 = (pos1.x < pos2.x) ? pos1 : pos2;
            Point angleP2 = (pos1.x < pos2.x) ? pos2 : pos1;
            double difX = angleP2.x - angleP1.x;
            double difY = angleP2.y - angleP1.y;
            double sampleAngle = Math.atan2(difX, difY) * (180 / Math.PI) - 180;
            foundSampleRotations.add(sampleAngle);

            double imageCenterX = baseImage.size().width / 2.0;
            double imageCenterY = baseImage.size().height / 2.0;
            double inchPosX = (centerPix.x - imageCenterX) * PIX2INCHES;
            double inchPosY = (centerPix.y - imageCenterY) * -PIX2INCHES;
            Point centerInInches = new Point(inchPosX, inchPosY);
            foundSamplePositionsInches.add(centerInInches);
        }

        if (foundSamplePositionsInches.isEmpty()) {
            return null;
        }

        double closestDist = Double.MAX_VALUE;
        int bestIndex = 0;
        for (int i = 0; i < foundSamplePositionsInches.size(); i++) {
            double d = distance(foundSamplePositionsInches.get(i), new Point(0, 0));
            if (d < closestDist) {
                closestDist = d;
                bestIndex = i;
            }
        }

        return new Sample(foundSamplePositionsInches.get(bestIndex), foundSampleRotations.get(bestIndex));
    }

    private double distance(Point one, Point two) {
        return Math.hypot(two.x - one.x, two.y - one.y);
    }

}
