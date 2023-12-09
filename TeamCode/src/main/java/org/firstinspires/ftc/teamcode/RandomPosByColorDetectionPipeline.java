package org.firstinspires.ftc.teamcode;

//import org.checkerframework.checker.units.qual.C;
//import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class RandomPosByColorDetectionPipeline extends OpenCvPipeline {
    //Telemetry telemetry;
    Mat mat = new Mat();
    public enum Location {LEFT, CENTER, RIGHT, NOT_FOUND}
    public Location location;
    //public RandomPosByColorDetectionPipeline(Telemetry t) { telemetry = t; }
    static final int STREAM_WIDTH = 1280; // modify for your camera
    static final int STREAM_HEIGHT = 720; // modify for your camera

    //Rectangle Sizes
    static final int WidthRectSides = 250;
    static final int HeightRectSides = 450;
    static final int WidthRectCenter = 450;
    static final int HeightRectCenter = 250;

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

    public double leftValue;
    public double centerValue;
    public double rightValue;

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(0, 0, 240);
        Scalar highHSV = new Scalar(255, 5, 255);

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat left = mat.submat(LEFT_ROI);
        Mat center = mat.submat(CENTER_ROI);
        Mat right = mat.submat(RIGHT_ROI);

        leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        centerValue = Core.sumElems(center).val[0] / CENTER_ROI.area() / 255;
        rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;

        left.release();
        center.release();
        right.release();

        /*telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
        telemetry.addData("Center raw value", (int) Core.sumElems(center).val[0]);
        telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);
        telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
        telemetry.addData("Center percentage", Math.round(centerValue * 100) + "%");
        telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");*/

        //boolean posLeft = leftValue > PERCENT_WHITE_THRESHOLD;
        //boolean posCenter = centerValue > PERCENT_WHITE_THRESHOLD;
        //boolean posRight = rightValue > PERCENT_WHITE_THRESHOLD;

        if (leftValue > PERCENT_WHITE_THRESHOLD & leftValue > rightValue & leftValue > centerValue){
            location = Location.LEFT;
            //telemetry.addData("Location", "left");
        }
        else if (centerValue > PERCENT_WHITE_THRESHOLD & centerValue > leftValue & centerValue > rightValue){
            location = Location.CENTER;
            //telemetry.addData("Location", "Center");
        }
        else if (rightValue > PERCENT_WHITE_THRESHOLD & rightValue > leftValue & rightValue > centerValue){
            location = Location.RIGHT;
            //telemetry.addData("Location", "Right");
        }
        else{
            location = Location.NOT_FOUND;
            //telemetry.addData("Location", "Not found");
        }
        //telemetry.update();

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
