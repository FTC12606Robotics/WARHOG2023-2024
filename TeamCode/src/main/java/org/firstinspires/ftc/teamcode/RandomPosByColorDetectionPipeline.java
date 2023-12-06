package org.firstinspires.ftc.teamcode;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class RandomPosByColorDetectionPipeline extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    public enum Location {LEFT, CENTER, RIGHT, NOT_FOUND}
    private Location location;

    static final int STREAM_WIDTH = 1280; // modify for your camera
    static final int STREAM_HEIGHT = 720; // modify for your camera

    //Rectangle Sizes
    static final int WidthRectSides = 300;
    static final int HeightRectSides = 500;
    static final int WidthRectCenter = 300;
    static final int HeightRectCenter = 500;

    //Change values here to correctly target the thirds
    static final Point RectLeftTopLeftAnchor = new Point((STREAM_WIDTH - WidthRectSides) / 2 + 000, ((STREAM_HEIGHT - HeightRectSides) / 2) - 100);
    static final Point RectCenterTopLeftAnchor = new Point((STREAM_WIDTH - WidthRectCenter) / 2 + 150, ((STREAM_HEIGHT - HeightRectCenter) / 2) - 100);
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
    static double PERCENT_WHITE_THRESHOLD = 0.1;

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(0, 0, 240);
        Scalar highHSV = new Scalar(255, 255, 255);

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat left = mat.submat(LEFT_ROI);
        Mat center = mat.submat(CENTER_ROI);
        Mat right = mat.submat(RIGHT_ROI);

        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double centerValue = Core.sumElems(center).val[0] / CENTER_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;

        left.release();
        center.release();
        right.release();

        telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
        telemetry.addData("Center raw value", (int) Core.sumElems(center).val[0]);
        telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);
        telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
        telemetry.addData("Center percentage", Math.round(centerValue * 100) + "%");
        telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");

        boolean posLeft = leftValue > PERCENT_WHITE_THRESHOLD;
        boolean posCenter = centerValue > PERCENT_WHITE_THRESHOLD;
        boolean posRight = rightValue > PERCENT_WHITE_THRESHOLD;

        if (posLeft){
            location = Location.LEFT;
            telemetry.addData("Location", "left");
        }
        else if (posCenter){
            location = Location.CENTER;
            telemetry.addData("Location", "Center");
        }
        else if (posRight){
            location = Location.RIGHT;
            telemetry.addData("Location", "Right");
        }
        else{
            location = Location.NOT_FOUND;
            telemetry.addData("Location", "Not found");
        }
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Imgproc.rectangle(mat, LEFT_ROI, new Scalar(0,255,0), 2);
        Imgproc.rectangle(mat, CENTER_ROI, new Scalar(0,0,255), 2);
        Imgproc.rectangle(mat, RIGHT_ROI, new Scalar(255,0,0), 2);

        return input;
    }

    public Location getLocation() {
        return location;
    }
}
