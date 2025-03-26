package org.firstinspires.ftc.teamcode.lib.vision;

import org.firstinspires.ftc.teamcode.lib.Constants;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class SampleDetectionPipeline extends OpenCvPipeline {
    private final double YellowHL = 12;
    private final double YellowHH = 61;
    private final double YellowSL = 119;
    private final double YellowSH = 255;
    private final double YellowVL = 186;
    private final double YellowVH = 255;

// Thresholding values for the Red Samples
    private final double Red1HL = 160;
    private final double Red1HH = 180;
    private final double Red1SL = 60;
    private final double Red1SH = 255;
    private final double Red1VL = 147;
    private final double Red1VH = 255;

    private final double Red2HL = 0;
    private final double Red2HH = 8;
    private final double Red2SL = 60;
    private final double Red2SH = 255;
    private final double Red2VL = 147;
    private final double Red2VH = 255;

// Thresholding values for the Blue Samples
    private final double BlueHL = 75;
    private final double BlueHH = 141;
    private final double BlueSL = 108;
    private final double BlueSH = 255;

    private final double BlueVL = 50;
    private final double BlueVH = 255;

    // Списки для хранения найденных семплов в pixels, inches и rotations
    private List<Point> foundSamplePositionsPix = new ArrayList<>();
    private List<Point> foundSamplePositionsInches = new ArrayList<>();
    private List<Double> foundSampleRotations = new ArrayList<>();

    // Класс для хранения данных семпла
    public class Sample {
        private Point centerInInches;
        private double rotation;

        public Sample(Point centerInInches, double rotation) {
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

        if (bestSample != null) {
            Imgproc.putText(input, "Angle: " + bestSample.getRotation(), new Point(50, 50),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(0, 255, 0), 2);
        }

        return input;
    }

    private Mat preprocessFrame(Mat frame) {
        Mat hsvFrame = new Mat();
        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

        // TODO: make commands to swap the tracking color
        Scalar lowerBound = new Scalar(Red1HL, Red1SL, Red1VL);
        Scalar upperBound = new Scalar(Red1HH, Red1SH, Red1VH);

        Mat boundMask = new Mat();
        Core.inRange(hsvFrame, lowerBound, upperBound, boundMask);

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
            if (arcLen < Constants.Vision.MIN_CONTOUR_LENGTH || arcLen > Constants.Vision.MAX_CONTOUR_LENGTH) {
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
                sumX += pt.x;
                sumY += pt.y;
            }
            int avgX = sumX / pts.size();
            int avgY = sumY / pts.size();
            Point centerPix = new Point(avgX, avgY);

            boolean originalPos = true;
            for (Point s : foundSamplePositionsPix) {
                if (distance(centerPix, s) < Constants.Vision.MIN_DISTANCE) {
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
            double inchPosX = (centerPix.x - imageCenterX) * Constants.Vision.PIX2INCHES;
            double inchPosY = (centerPix.y - imageCenterY) * -Constants.Vision.PIX2INCHES;
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

        Imgproc.circle(baseImage, foundSamplePositionsPix.get(bestIndex), 25, new Scalar(0, 255, 0), 9);

        return new Sample(foundSamplePositionsInches.get(bestIndex), foundSampleRotations.get(bestIndex));
    }

    private double distance(Point one, Point two) {
        return Math.hypot(two.x - one.x, two.y - one.y);
    }
}
