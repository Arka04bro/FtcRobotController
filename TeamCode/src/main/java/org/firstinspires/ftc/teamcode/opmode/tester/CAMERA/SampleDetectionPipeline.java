package org.firstinspires.ftc.teamcode.opmode.tester.CAMERA;

// imports

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.opencv.core.Mat;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

/**
 * This class is designed for you to use it an image from the camera on the end of the arm and
 * it will return the position and rotation of the best sample to grab it could find
 * ( the one that is closest to the center of the image)
 */
public class SampleDetectionPipeline extends SampleDetectionProcessor {


// Thresholding values for Yellow Samples

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

// Edge detection constants for the seperation canny

    private final double CANNY_LOW_Y = 272;
    private final double CANNY_HIGH_Y = 27;

    private final double CANNY_LOW_R = 1542;
    private final double CANNY_HIGH_R = 10;

    private final double CANNY_LOW_B = 272;
    private final double CANNY_HIGH_B = 27;
// Contour filtering constants

    // all detected contours must be within these sizes
    private final int MIN_CONTOUR_LENGTH = 290;
    private final int MAX_CONTOUR_LENGTH = 675;

    // allowed distance between the detected contours
    private final double MIN_DISTANCE = 65;

    // unit conversions
    private final double PIX2INCHES = 0.015;
    // 3 inches / 200 pix

    private final double INCH2PIX = 66.667;
    // 200 pix / 3 inches


// Other constants


    // the amount the image should be cropped by, more means less image at the end
    private final double CROPPING_FACTOR = 2;

    // should just be something kinda hacky but could be a useful thing to tune
    private final double EXTRA_CROPPING_X = 0;
    private final double EXTRA_CROPPING_Y = 0;


    // a list of the pixel positions on the cropped frame of all found samples. type of OpenCV Point = (0,0)
    private List<Point> foundSamplePositionsPix = new ArrayList<>();

    // same thing as the list above but these are the positions in inches and relitive to the center of the frame
    private List<Point> foundSamplePositionsInches = new ArrayList<>();

    // a list of all the rotations all all found samples. the order of the rotations lines up with the order of the points above.
    private List<Double> foundSampleRotations = new ArrayList<>();


// mat definitions, but here so we can release their data after use

    Mat blurred = new Mat();
    Mat threshFlipped = new Mat();

    Mat threshBGR = new Mat();
    Mat eroded = new Mat();
    ;
    Mat seperationCanny = new Mat();
    Mat dilatedSeperationCanny = new Mat();
    Mat blurredCanny = new Mat();
    Mat BGR_Blurred_Canny = new Mat();
    Mat outlineCanny = new Mat();
    Mat outlineBlurred = new Mat();
    Mat hierarchy = new Mat();


    // other things I added later.. I am tired give me a break

    // a telemetry instance from the FIRST SDK
    private Telemetry telemetry = null;

    /**
     * an int used to decide which color of sample we should be looking for
     * 0 = yellow samples
     * 1 = red samples
     * 2 = blue samples
     * */


    // a boolean used to tell the algorithm whether or not it should be running. when true the algorithm runs, else it does not. only have this true when you need it.


    /**
     * the contructor of this pipeline
     *
     * @param telemetry the telemetry instance
     */
    public SampleDetectionPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }


    /**
     * The init function overrided from the OpenCvPipeline class
     * nothing is in here as we don't need stuff in here but it is a requirement to have this function exist
     */
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // Executed before the first call to processFrame
    }

    /**
     * ran by the OpenCVPipeline class and it passes on Mat input every frame the vision portal is enabled
     */
    @Override
    public Mat processFrame(Mat input, long captureTimeNanos) {

        // releases the data from all Mats we use
        clearMats();
        // Executed every time a new frame is dispatched

        // the output image we return at the endof the processFrame function
        Mat output = new Mat();

        // sets the foundSample var to false to ensure we don't say we have a sample when we don't
        foundSample = false;

        // checks if we should run the algorithm
        if (update) {
            // sets the output image to the output of the main algorithm
            // passes in the input image from OpenCV and the colorMode (0-2) for which color of samples we are looking for
            output = findSamples(input, colorMode);
        } else {
            // if we should not update, we still need to return something so we just return the input as the output
            output = input;
        }


        // you can use this code the read out to the user where the sample we found is. only run this code if there is a sample found, so check first!
        // telemetry.addData("found sample:", foundSample);
        // if (foundSample){
        //     telemetry.addData("found position X", foundSamplePositionX);
        //     telemetry.addData("found position Y", foundSamplePositionY);
        //     telemetry.addData("found position Yaw", foundSamplePositionYaw);
        // }

        // telemetry.update();


        return output; // Return the image that will be displayed in the viewport on the driver hub
    }

    // like the init function, needed by the OpenCVPipeline... but we don't really use it
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }


    /**
     * release the data from the Mats we use to help save RAM
     */
    public void clearMats() {

        blurred.release();
        threshFlipped.release();

        threshBGR.release();
        eroded.release();
        seperationCanny.release();
        dilatedSeperationCanny.release();
        blurredCanny.release();
        BGR_Blurred_Canny.release();
        outlineCanny.release();
        outlineBlurred.release();
        hierarchy.release();
    }


    /**
     * The main algorithm
     * <p>
     * In here we do a lot of steps:
     * correct for the lens distortion
     * blur the image
     * crop the image down
     * convert the color space from RGB to HSV
     * Threshhold the image
     * flip that result to get a mask
     * convert that gray scale masm to to HSV through RGB
     * eroded that mask to allow for more detail
     * subtract the mask from the cropped image to just get the correct color of pixels
     * edge detection to slip all of the samples apart
     * dilate those seperation edges
     * also subtract those
     * finds all of the contours of the seperated samples
     * filters through all of the contours
     * ensures that the lengths are within the size of a sample are correct
     * approximates the contours into only 4 points
     * finds the average position of all four corners
     * finds the longest side of the sample
     * uses that side to find the angle
     * chooses the closest sample to the center.
     *
     * @param img  the input image
     * @param mode which color of samples we should find (0-2) yellow, red, blue
     * @return an image of the seperated samples with a green circle around the one we want to use
     */
    private Mat findSamples(Mat img, int mode) {

        // sets the arrays back to empty since the last update
        foundSamplePositionsPix = new ArrayList<>();
        foundSamplePositionsInches = new ArrayList<>();
        foundSampleRotations = new ArrayList<>();
        foundSample = false;

        // corrects the lens distortion of the image
        Mat corrected = undistortImage(img);


        // blurs the image with a kernel of (3,3)
        Imgproc.GaussianBlur(corrected, blurred, new Size(3, 3), 5);

        // crops the image
        Mat cropped = crop(blurred);

        // converts the images color space to from RGB to HSV
        Mat hsv = convertColorSpace(cropped);

        // thresholds the image
        Mat thresholded = thresholdImage(hsv, mode);

        // flips the threshhold to make a make
        Core.bitwise_not(thresholded, threshFlipped);

        // makes the make in the same color space as the base image
        Imgproc.cvtColor(threshFlipped, threshBGR, Imgproc.COLOR_GRAY2RGB);
        Mat threshHSV = convertColorSpace(threshBGR);

        // erodes the mask
        Mat kernel = Mat.ones(5, 5, 0);
        Imgproc.erode(threshHSV, eroded, kernel);

        Mat baseImage = hsv;

        // uses that mask to remove all colors that are not the target samples
        // Set all black regions to black : written by copilot :(
        for (int row = 0; row < baseImage.rows(); row++) {
            for (int col = 0; col < baseImage.cols(); col++) {
                if (eroded.get(row, col)[2] > 0) {
                    double[] pixel = {0, 0, 0}; // Black in HSV
                    baseImage.put(row, col, pixel);
                }
            }
        }

        // seperates the samples based on the color, different samples needed different tuning
        if (colorMode == 0) {
            Imgproc.Canny(baseImage, seperationCanny, CANNY_LOW_Y, CANNY_HIGH_Y);
        } else if (colorMode == 1) {
            Imgproc.Canny(baseImage, seperationCanny, CANNY_LOW_R, CANNY_HIGH_R);

        } else if (colorMode == 2) {
            Imgproc.Canny(baseImage, seperationCanny, CANNY_LOW_B, CANNY_HIGH_B);
        }

        // dilates that canny
        Mat dilationKernal = Mat.ones(4, 4, CvType.CV_32F);
        Imgproc.dilate(seperationCanny, dilatedSeperationCanny, dilationKernal);

        // blurs the dilation result
        Imgproc.GaussianBlur(seperationCanny, blurredCanny, new Size(3, 3), 3);

        // seperates the samples out if they are touching
        Mat seperated = baseImage;
        Imgproc.cvtColor(blurredCanny, BGR_Blurred_Canny, Imgproc.COLOR_GRAY2RGB);

        Mat HSV_Blurred_Canny = convertColorSpace(BGR_Blurred_Canny);

        for (int row = 0; row < baseImage.rows(); row++) {
            for (int col = 0; col < baseImage.cols(); col++) {
                if (
                        HSV_Blurred_Canny.get(row, col)[2] > 0) {
                    double[] pixel = {0, 0, 0}; // Black in HSV
                    seperated.put(row, col, pixel);
                }
            }
        }


        // gets the contours of the samples
        Imgproc.Canny(seperated, outlineCanny, 272, 70);
        Imgproc.GaussianBlur(outlineCanny, outlineBlurred, new Size(3, 3), 3);
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(outlineBlurred, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        // now that we have those contours, we filter through them one by one!
        for (int i = 0; i < contours.size(); i++) {


            // converts the contours to listed out contours
            MatOfPoint2f k = new MatOfPoint2f(contours.get(i).toArray());

            // verifies the length of the contour is valid, else we move on to the next contour in the for loop
            if (Imgproc.arcLength(k, true) < MIN_CONTOUR_LENGTH) {
                continue;
            }
            if (Imgproc.arcLength(k, true) > MAX_CONTOUR_LENGTH) {
                continue;
            }

            // approximates the contour down to 4 points
            MatOfPoint2f approxContour = new MatOfPoint2f();
            Imgproc.approxPolyDP(k, approxContour, Imgproc.arcLength(k, true) / 22, true);

            //  all four points in the contour
            List<Point> pointsList = approxContour.toList();
            MatOfPoint convertedApproxContour = new MatOfPoint();
            convertedApproxContour.fromList(pointsList);
            MatOfPoint finalApproxContour = new MatOfPoint();


// /**
//  * The logic needed to approximate more if the size has more than 4 points
//  * Not used in the most recent python algorithm so I commented it out but will still leave it here as it took me a while lol
//  */

// //            if (convertedApproxContour.size().width > 4){
// //                MatOfPoint2f tempReApprox = null;
// //                Imgproc.approxPolyDP(approxContour, tempReApprox, Imgproc.arcLength(approxContour, true) / 10, true);
// //                List<Point> morePoints = tempReApprox.toList();
// //                finalApproxContour.fromList(morePoints);
// //            }else{
            finalApproxContour = convertedApproxContour;
// //            }

            // for each position in the approximated contour, indexing
            int p = 0;

            // lists of the positions x and y in pixels of the 4 contour position
            List<Integer> positionsX = new ArrayList<>();
            List<Integer> positionsY = new ArrayList<>();

            // adds these points to the lists
            for (Point pos : finalApproxContour.toList()) {
                positionsX.add((int) pos.x);
                positionsY.add((int) pos.y);
                p++;
            }

            //averages the positions out to find the center
            int avgX = avgInt(positionsX);
            int avgY = avgInt(positionsY);

            // combines the averages to get the center point
            Point centorPos = new Point(avgX, avgY);

            // now we check if the contour we have is the first in its position, sometimes there are contours that overlap!
            boolean originalPos = true;

            for (Point s : foundSamplePositionsPix) {
                if (distance(centorPos, s) < MIN_DISTANCE) {
                    originalPos = false;
                }
            }

            // if the position is original, we final the detection!
            if (originalPos) {
                // adds the found center position to our list of found positions this frame in pixels!
                foundSamplePositionsPix.add(centorPos);

                // The comments will be continued another day :) - Tobin
                double bestDistance = 0.0;
                int bestY = 0;
                int y = 0;
                Point lastPos = new Point();
                Point pos1 = new Point();
                Point pos2 = new Point();

                for (Point pos : finalApproxContour.toList()) {
                    if (y == 0) {
                        lastPos = pos;
                        y++;
                        continue;
                    }
                    double dist = distance(pos, lastPos);
                    if (dist > bestDistance) {
                        bestDistance = dist;
                        bestY = y;
                        pos1 = pos;
                        pos2 = lastPos;
                    }
                    lastPos = pos;
                    y++;
                }
                Point anglePos1 = new Point();
                Point anglePos2 = new Point();

                if (pos1.x < pos2.x) {
                    anglePos1 = pos1;
                    anglePos2 = pos2;
                } else {
                    anglePos1 = pos2;
                    anglePos2 = pos1;
                }

                double difX = anglePos2.x - anglePos1.x;
                double difY = anglePos2.y - anglePos1.y;

                double sampleAngle = Math.atan2(difX, difY) * (180 / Math.PI) - 180;

                double imageCenterX = baseImage.size().width / 2;
                double imageCenterY = baseImage.size().height / 2;

                // telemetry.addLine(""+ centorPos);

                // telemetry.addLine(""+ PIX2INCHES);

                double inchPositionX = (centorPos.x - imageCenterX) * PIX2INCHES;
                double inchPositionY = (centorPos.y - imageCenterY) * -PIX2INCHES;

                // telemetry.addLine(""+ inchPositionX + " : " + inchPositionY);

                Point foundPosition = new Point(inchPositionX, inchPositionY);
                // telemetry.addLine(""+ baseImage.size());
                foundSamplePositionsInches.add(foundPosition);
                foundSampleRotations.add(sampleAngle);
            }


        }

        // then we need to find which sample out of all of the samples we found we want to grab!
        // right now we just calculate which one is closest.

        double closestDist = 1000000;
        int bestI = 0;

        for (int i = 0; i < foundSamplePositionsInches.size(); i++) {

            // if we get here then we definitely found a sample!
            foundSample = true;
            // telemetry.addLine(""+ closestDist);
            if (distance(foundSamplePositionsInches.get(i), new Point(0, 0)) < closestDist) {
                closestDist = distance(foundSamplePositionsInches.get(i), new Point(0, 0));
                bestI = i;
            }
        }
        Imgproc.cvtColor(baseImage, baseImage, Imgproc.COLOR_HSV2RGB);
        if (foundSample) {
            // telemetry.addLine(""+ bestI);
            // telemetry.addLine(""+ foundSamplePositionsInches);
            // telemetry.addLine(""+ foundSampleRotations);


            foundSamplePositionX = foundSamplePositionsInches.get(bestI).x;
            foundSamplePositionY = foundSamplePositionsInches.get(bestI).y;
            foundSamplePositionYaw = foundSampleRotations.get(bestI);

            Imgproc.circle(baseImage, foundSamplePositionsPix.get(bestI), 25, new Scalar(0, 255, 0), 9);

        }

        return baseImage;
        // return seperated;
    }


    private Mat thresholdImage(Mat src, int mode) {
        Scalar lowerBound = null;
        Scalar upperBound = null;

        Mat dst = new Mat();
        ;

        if (mode == 0) {
            // yellow

            // create the yellow thresholds
            lowerBound = new Scalar(YellowHL, YellowSL, YellowVL);
            upperBound = new Scalar(YellowHH, YellowSH, YellowVH);

            // threshold the image!
            Core.inRange(src, lowerBound, upperBound, dst);

        } else if (mode == 1) {
            //red

            //since red is the color where the HSV color space wraps around, we need to use two different thresholds to get all red pixels

            // create the red thresholds
            lowerBound = new Scalar(Red1HL, Red1SL, Red1VL);
            upperBound = new Scalar(Red1HH, Red1SH, Red1VH);

            Scalar lowerBound2 = new Scalar(Red2HL, Red2SL, Red2VL);
            Scalar upperBound2 = new Scalar(Red2HH, Red2SH, Red2VH);
            Mat firstRed = new Mat();
            Mat secondRed = new Mat();

            // threshold the image!
            Core.inRange(src, lowerBound, upperBound, firstRed);

            Core.inRange(src, lowerBound2, upperBound2, secondRed);

            Core.add(firstRed, secondRed, dst);


        } else if (mode == 2) {
            //blue

            // create the red thresholds
            lowerBound = new Scalar(BlueHL, BlueSL, BlueVL);
            upperBound = new Scalar(BlueHH, BlueSH, BlueVH);

            // threshold the image!
            Core.inRange(src, lowerBound, upperBound, dst);

        }
        return dst;
    }

    private Mat crop(Mat src) {
        int scaleX = src.cols();
        int scaleY = src.rows();

        int cropCornerX = (int) (scaleX / (CROPPING_FACTOR * 2));
        int cropCornerY = (int) (scaleY / (CROPPING_FACTOR * 2));

        int rectWidth = (int) (scaleX / (CROPPING_FACTOR));
        int rectHeight = (int) (scaleY / (CROPPING_FACTOR));

        // calculating the part of the image we want to remain
        Rect roi = new Rect(cropCornerX, cropCornerY, rectWidth, rectHeight);

        Mat cropped = new Mat(src, roi);


        return cropped;
    }

    private Mat convertColorSpace(Mat src) {
        Mat hsv = new Mat();
        ;
        Imgproc.cvtColor(src, hsv, Imgproc.COLOR_RGB2HSV);
        return hsv;
    }

    private Mat undistortImage(Mat src) {
        Mat cameraMatrix = new Mat(3, 3, CvType.CV_64F);

        //These are the values gotten from the camera calibration we did

        // assign the values to the matrix... I could not find a better way through the documentation
        cameraMatrix.put(0, 0, 600.01851744);
        cameraMatrix.put(0, 1, 0);
        cameraMatrix.put(0, 2, 906.817157357);
        cameraMatrix.put(1, 0, 0);
        cameraMatrix.put(1, 1, 600.01851744);
        cameraMatrix.put(1, 2, 516.73047402);
        cameraMatrix.put(2, 0, 0);
        cameraMatrix.put(2, 1, 0);
        cameraMatrix.put(2, 2, 1);

        // assigning values to the camera distortion coefficients... the only way I know how
        Mat dist_coeffs = new Mat(1, 5, CvType.CV_64F);

        dist_coeffs.put(0, 0, 0.0115588983608);
        dist_coeffs.put(0, 1, -0.0313347203804);
        dist_coeffs.put(0, 2, 0.00013459478315);
        dist_coeffs.put(0, 3, 0.000897741867319);
        dist_coeffs.put(0, 4, 0.00542752872672);

        // create the destination for the undistorted image
        Mat unDistorted = new Mat();

        // un distort
        Calib3d.undistort(src, unDistorted, cameraMatrix, dist_coeffs);
        return unDistorted;
    }

    private Mat rescaleFrame(Mat frame, double scale) {
        double width = (int) Math.round(frame.cols() * scale);
        double height = (int) Math.round(frame.rows() * scale);
        Size image_size = new Size(width, height);
        Mat dst = new Mat();
        ;
        Imgproc.resize(frame, dst, image_size);
        return dst;
    }

    private int avgInt(List<Integer> numbers) {
        double count = 0;
        int i = 0;
        for (int n : numbers) {
            count += n;
            i += 1;
        }
        if (i != 0) {
            count /= i;
            int avg = (int) count;
            return avg;
        } else {
            return -1;
        }
    }

    private double distance(Point one, Point two) {
        return Math.sqrt(Math.pow(two.x - one.x, 2) + Math.pow(two.y - one.y, 2));
    }

}
