package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.Gamepad;

@Autonomous(name="WARHOGAutoPushBot", group="")
public class WARHOGAutoPushBot extends LinearOpMode {

    public WARHOGAutoPushBot() throws InterruptedException {}

    private StartPosColor startPosColor = StartPosColor.RED;
    private enum StartPosColor {RED, BLUE};
    private StartPosPosition startPosPosition = StartPosPosition.FRONT;
    private enum StartPosPosition {FRONT, BACK};

    //OpenCvCamera camera;
    //AprilTagDetectionPipeline aprilTagDetectionPipeline;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    static final double FEET_PER_METER = 3.28084;

    //int colorMod = 0;
    //int posMod = 0;

    boolean front=false, back=false, red=false, blue=false; //Bools to set position

    double speed = .50;
    double startSleep = 1; //How many seconds to wait before starting autonomous

    //This stuff does not need to be changed
    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
/*
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    double tagsize = 0.166;

    //tag ID 1,2,3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    int ID_TAG_OF_INTEREST = 18;

    AprilTagDetection tagOfInterest = null;
*/

    @Override
    public void runOpMode() throws InterruptedException {

        PushbotDrivetrain pushDrivetrain = new PushbotDrivetrain(hardwareMap, telemetry);
        //Intake intake = new Intake(hardwareMap, telemetry);
        //Outtake outtake = new Outtake(hardwareMap, telemetry);


        /*int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

         */

        telemetry.setMsTransmissionInterval(50);

        //init loop
        while (!isStarted() && !isStopRequested()) {
            //intake.runArm(Intake.Height.STARTSIZING);
            //set up inputs - have previous so that you can check rising edge
            try {
                previousGamepad1.copy(currentGamepad1);
                previousGamepad2.copy(currentGamepad2);

                currentGamepad1.copy(gamepad1);
                currentGamepad2.copy(gamepad2);
            }
            catch (Exception e) {
                // Swallow the possible exception, it should not happen as
                // currentGamepad1/2 are being copied from valid Gamepads.
            }

            //set up initialization procedures
            if (currentGamepad1.b) {
                startPosColor = StartPosColor.RED;
            }
            if (currentGamepad1.x) {
                startPosColor = StartPosColor.BLUE;
            }
            if (currentGamepad1.dpad_down) {
                startPosPosition = StartPosPosition.BACK;
            }
            if (currentGamepad1.dpad_up) {
                startPosPosition = StartPosPosition.FRONT;
            }

            //Override speed with driver hub
            if(currentGamepad1.y && !previousGamepad1.y){
                speed+=.05;
            }
            if(currentGamepad1.a && !previousGamepad1.a){
                speed-=.05;
            }
            if(speed>1){
                speed=1;
            }
            if(speed<.4){
                speed=.4;
            }

            //Override startSleep with driver hub
            if(currentGamepad1.dpad_right && !previousGamepad1.dpad_right){
                startSleep+=.5;
            }
            if(currentGamepad1.dpad_left && !previousGamepad1.dpad_left){
                startSleep-=.5;
            }
            if(startSleep>20){
                startSleep=20;
            }
            if(startSleep<1){
                startSleep=1;
            }

            telemetry.addData("Color", startPosColor);
            telemetry.addData("Position", startPosPosition);
            telemetry.addData("Speed", speed);
            telemetry.addData("startSleep", startSleep);

            /*ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            //detect apriltags
            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }*/

            telemetry.update();
            sleep(20);
        }


        // start command just came in

        //set modifier values
        switch (startPosColor){
            case RED:
                //colorMod = 1;
                red=true;
                break;
            case BLUE:
                //colorMod = -1;
                blue=true;
                break;
        }
        switch (startPosPosition){
            case FRONT:
                //posMod = -1;
                front=true;
                break;
            case BACK:
                //posMod = 1;
                back=true;
                break;
        }

        //2023-2024 Autonomous Main Code
        //Blocks to run for different start positions
        if(red&&front){
            sleep((long)((startSleep)*1000));
            //Turn and Move
            pushDrivetrain.RotateForDegree(90, speed-.25);
            pushDrivetrain.MoveForDis(96, speed);

            telemetry.addLine("Park complete");
            telemetry.update();
        }
        else if(red&&back){
            sleep((long)(startSleep*1000));
            //Turn and Move
            pushDrivetrain.RotateForDegree(-90, speed-.25);
            pushDrivetrain.MoveForDis(48, .25);

            telemetry.addLine("Park complete");
            telemetry.update();
        }
        else if(blue&&front){
            sleep((long)((startSleep)*1000));
            //Turn and Move
            pushDrivetrain.RotateForDegree(90, speed-.25);
            pushDrivetrain.MoveForDis(96, speed);

            telemetry.addLine("Park complete");
            telemetry.update();
        }
        else if(blue&&back){
            sleep((long)(startSleep*1000));
            //Turn and Move
            pushDrivetrain.RotateForDegree(-90, speed-.25);
            pushDrivetrain.MoveForDis(48, .25);

            telemetry.addLine("Park complete");
            telemetry.update();
        }


    /*void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        /*telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));

    }*/

    }
    private void RunMotorsForSeconds(double secs, double power) throws InterruptedException{
        PushbotDrivetrain pushDrivetrain = new PushbotDrivetrain(hardwareMap, telemetry);
        pushDrivetrain.setMotorPowers(power, power, power,power);
        sleep((long)(secs*1000));
        pushDrivetrain.setMotorPowers(0,0,0,0);
    }
}