package org.firstinspires.ftc.teamcode;

import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class RandomPosByColorDetectionPipeline extends OpenCvPipeline {
    Mat mat = new Mat();
    public enum Location {LEFT, CENTER, RIGHT, NOT_FOUND}
    public Location location;

    public enum Color {WHITE, RED, BLUE}
    public Color color = Color.WHITE;

    static final int STREAM_WIDTH = 1280; // modify for your camera
    static final int STREAM_HEIGHT = 720; // modify for your camera

    //Rectangle Sizes
    static final int WidthRectSides = 300;
    static final int HeightRectSides = 450;
    static final int WidthRectCenter = 450;
    static final int HeightRectCenter = 300;

    //Change values here to correctly target the thirds
    static final Point RectLeftTopLeftAnchor = new Point((STREAM_WIDTH - WidthRectSides) / 2 - 450, ((STREAM_HEIGHT - HeightRectSides) / 2) - 100);
    static final Point RectCenterTopLeftAnchor = new Point((STREAM_WIDTH - WidthRectCenter) / 2 + 0, ((STREAM_HEIGHT - HeightRectCenter) / 2) - 200);
    static final Point RectRightTopLeftAnchor = new Point((STREAM_WIDTH - WidthRectSides) / 2 + 450, ((STREAM_HEIGHT - HeightRectSides) / 2) - 100);

    static Point RectLeftTLCorner = new Point(RectLeftTopLeftAnchor.x, RectLeftTopLeftAnchor.y);
    static Point RectLeftBRCorner = new Point(RectLeftTopLeftAnchor.x + WidthRectSides, RectLeftTopLeftAnchor.y + HeightRectSides);

    static Point RectCenterTLCorner = new Point(RectCenterTopLeftAnchor.x, RectCenterTopLeftAnchor.y);
    static Point RectCenterBRCorner = new Point(RectCenterTopLeftAnchor.x + WidthRectCenter, RectCenterTopLeftAnchor.y + HeightRectCenter);

    static Point RectRightTLCorner = new Point(RectRightTopLeftAnchor.x, RectRightTopLeftAnchor.y);
    static Point RectRightBRCorner = new Point(RectRightTopLeftAnchor.x + WidthRectSides, RectRightTopLeftAnchor.y + HeightRectSides);

    static final Rect LEFT_ROI = new Rect(RectLeftTLCorner, RectLeftBRCorner);
    static final Rect CENTER_ROI = new Rect(RectCenterTLCorner, RectCenterBRCorner);
    static final Rect RIGHT_ROI = new Rect(RectRightTLCorner, RectRightBRCorner);
    static double PERCENT_WHITE_THRESHOLD = 0.01;
    static double PERCENT_RED_THRESHOLD = 0.10;
    static double PERCENT_BLUE_THRESHOLD = 0.10;

    public double leftValue;
    public double centerValue;
    public double rightValue;

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        //For the White pixel color
        Scalar whiteLowHSV = new Scalar(0, 0, 240);
        Scalar whiteHighHSV = new Scalar(255, 5, 255);
        //For the Red color
        Scalar redLowHSV = new Scalar(0, 60, 100);
        Scalar redHighHSV = new Scalar(255, 255, 255);
        //For the Blue color
        Scalar blueLowHSV = new Scalar(100, 100, 100);
        Scalar blueHighHSV = new Scalar(185, 255, 255);

        if (color == Color.WHITE){
            Core.inRange(mat, whiteLowHSV, whiteHighHSV, mat);
        }
        else if (color == Color.RED){
            Core.inRange(mat, redLowHSV, redHighHSV, mat);
        }
        else if (color == Color.BLUE){
            Core.inRange(mat, blueLowHSV, blueHighHSV, mat);
        }

        Mat left = mat.submat(LEFT_ROI);
        Mat center = mat.submat(CENTER_ROI);
        Mat right = mat.submat(RIGHT_ROI);

        leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        centerValue = Core.sumElems(center).val[0] / CENTER_ROI.area() / 255;
        rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;

        left.release();
        center.release();
        right.release();

        //To sense the randomPos based on the color
        if (color == Color.WHITE){
            if (leftValue >= PERCENT_WHITE_THRESHOLD & leftValue > rightValue & leftValue > centerValue){
                location = Location.LEFT;
            }
            else if (centerValue >= PERCENT_WHITE_THRESHOLD & centerValue > leftValue & centerValue > rightValue){
                location = Location.CENTER;
            }
            else if (rightValue >= PERCENT_WHITE_THRESHOLD & rightValue > leftValue & rightValue > centerValue){
                location = Location.RIGHT;
            }
            else{
                location = Location.NOT_FOUND;
            }
        }
        else if (color == Color.RED){
            if (leftValue >= PERCENT_RED_THRESHOLD & leftValue > rightValue & leftValue > centerValue){
                location = Location.LEFT;
            }
            else if (centerValue >= PERCENT_RED_THRESHOLD & centerValue > leftValue & centerValue > rightValue){
                location = Location.CENTER;
            }
            else if (rightValue >= PERCENT_RED_THRESHOLD & rightValue > leftValue & rightValue > centerValue){
                location = Location.RIGHT;
            }
            else{
                location = Location.NOT_FOUND;
            }
        }
        else if (color == Color.BLUE){
            if (leftValue >= PERCENT_BLUE_THRESHOLD & leftValue > rightValue & leftValue > centerValue){
                location = Location.LEFT;
            }
            else if (centerValue >= PERCENT_BLUE_THRESHOLD & centerValue > leftValue & centerValue > rightValue){
                location = Location.CENTER;
            }
            else if (rightValue >= PERCENT_BLUE_THRESHOLD & rightValue > leftValue & rightValue > centerValue){
                location = Location.RIGHT;
            }
            else{
                location = Location.NOT_FOUND;
            }
        }

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Imgproc.rectangle(mat, RectLeftTLCorner, RectLeftBRCorner, new Scalar(0,255,0), 2);
        Imgproc.rectangle(mat, RectCenterTLCorner, RectCenterBRCorner, new Scalar(0,0,255), 2);
        Imgproc.rectangle(mat, RectRightTLCorner, RectRightBRCorner, new Scalar(255,0,0), 2);

        return mat;
    }

    public Location getLocation() {
        return location;
    }
}
