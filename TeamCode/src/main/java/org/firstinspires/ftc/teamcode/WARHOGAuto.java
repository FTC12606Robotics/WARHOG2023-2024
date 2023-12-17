package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

//import java.util.ArrayList;

@Autonomous(name="WARHOGAuto", group="")
public class WARHOGAuto extends LinearOpMode {

    public WARHOGAuto() throws InterruptedException {}

    private StartPosColor startPosColor = StartPosColor.RED;
    private enum StartPosColor {RED, BLUE}
    private StartPosPosition startPosPosition = StartPosPosition.BACK;
    private enum StartPosPosition {FRONT, BACK}
    private ParkPos parkPos = ParkPos.CORNER;
    private enum ParkPos {NO, CORNER, MIDDLE} //For where to park if at all
    private RandomPos randomPos = RandomPos.NULL;
    private enum RandomPos {NULL, LEFT, CENTER, RIGHT} //For what position the randomization is in
    private ActionCombination actionCombination = ActionCombination.PARK_ONLY;
    private enum ActionCombination {PARK_ONLY/*, BOARD_ONLY*/, SPIKE_ONLY, PARK_BOARD, NONE/*, SPIKE_BOARD*/, PARK_SPIKE, PARK_BOARD_SPIKE}

    OpenCvCamera camera;
    //AprilTagDetectionPipeline aprilTagDetectionPipeline;
    RandomPosByColorDetectionPipeline randomPosByColorDetectionPipeline;


    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    static final double FEET_PER_METER = 3.28084;

    int colorMod = 0;

    boolean front=false, back=false, red=false, blue=false; //Bools to set position

    boolean willPark = false; //for interior code use for if the robot will park
    boolean willSpike = false; //for interior code use for if the robot will place a pixel on a spike
    boolean willBoard = false; //for interior code use for if the robot will place a pixel on the backdrop
    boolean useCamera = true; //for testing to say if it will use the camera

    double speed = .50;
    double startSleep = 1; //How many seconds to wait before starting autonomous

    //this stuff does not need to be changed
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

        Drivetrain drivetrain = new Drivetrain(hardwareMap, telemetry);
        Intake intake = new Intake(hardwareMap, telemetry);

        //Setup Camera and OpenCV
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        randomPosByColorDetectionPipeline = new RandomPosByColorDetectionPipeline();

        //camera.setPipeline(aprilTagDetectionPipeline);
        camera.setPipeline(randomPosByColorDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1280,720, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode)
            {
                telemetry.addLine("Camera Failed to setup");
                telemetry.update();

                //To cover my butt, but it might come back to bite my butt
                randomPos = RandomPos.NULL;
            }
        });


        telemetry.setMsTransmissionInterval(50);

        //init loop
        while (!isStarted() && !isStopRequested()) {
            //Run the robot arm to its starting position
            intake.runArm(Intake.Height.STARTSIZING);

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
            if(speed<.3){
                speed=.3;
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
            if(startSleep<0){
                startSleep=0;
            }

            //To set where to park in backstage
            //***Maybe set a different button***
            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
                if(parkPos == ParkPos.MIDDLE){
                    parkPos = ParkPos.CORNER;
                }
                else if (parkPos == ParkPos.CORNER){
                    parkPos = ParkPos.NO;
                }
                else if (parkPos == ParkPos.NO){
                    parkPos = ParkPos.MIDDLE;
                }
            }

            //Go through different combinations of things to do and set bools
            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper){
                if(actionCombination == ActionCombination.PARK_ONLY){
                    actionCombination = ActionCombination.SPIKE_ONLY;
                    willPark = false;
                    willBoard = false;
                    willSpike = true;
                }
                /*else if(actionCombination == actionCombination.BOARD_ONLY){
                    actionCombination = actionCombination.SPIKE_ONLY;
                    willPark = false;
                    willBoard = false;
                    willSpike = true;
                }*/
                else if(actionCombination == ActionCombination.SPIKE_ONLY){
                    actionCombination = ActionCombination.PARK_BOARD;
                    willPark = true;
                    willBoard = true;
                    willSpike = false;
                }
                else if(actionCombination == ActionCombination.PARK_BOARD){
                    actionCombination = ActionCombination.PARK_SPIKE;
                    willPark = true;
                    willBoard = false;
                    willSpike = true;
                }
                /*else if(actionCombination == actionCombination.SPIKE_BOARD){
                    actionCombination = actionCombination.PARK_SPIKE;
                    willPark = true;
                    willBoard = false;
                    willSpike = true;
                }*/
                else if(actionCombination == ActionCombination.PARK_SPIKE){
                    actionCombination = ActionCombination.PARK_BOARD_SPIKE;
                    willPark = true;
                    willBoard = true;
                    willSpike = true;
                }
                else if(actionCombination == ActionCombination.PARK_BOARD_SPIKE){
                    actionCombination = ActionCombination.NONE;
                    willPark = false;
                    willBoard = false;
                    willSpike = false;
                }
                else if(actionCombination == ActionCombination.NONE){
                    actionCombination = ActionCombination.PARK_ONLY;
                    willPark = true;
                    willBoard = false;
                    willSpike = false;
                }
            }

            //Manually set the random pos for testing and/or if camera doesn't work
            if (currentGamepad1.right_stick_button && !previousGamepad1.right_stick_button){
                if (randomPos == RandomPos.NULL){
                    randomPos = RandomPos.LEFT;
                }
                else if (randomPos == RandomPos.LEFT){
                    randomPos = RandomPos.CENTER;
                }
                else if (randomPos == RandomPos.CENTER){
                    randomPos = RandomPos.RIGHT;
                }
                else if (randomPos == RandomPos.RIGHT){
                    randomPos = RandomPos.NULL;
                }
            }

            //For camera usage in decision making
            if (currentGamepad1.left_stick_button && !previousGamepad1.left_stick_button){
                useCamera = !useCamera;
            }

            //OpenCV Pipeline 2 w/ RandomPosByColorDetectionPipeline
            /*switch (randomPosByColorDetectionPipeline.getLocation()){
                case LEFT:
                    randomPos = RandomPos.LEFT;
                    break;
                case CENTER:
                    randomPos = RandomPos.CENTER;
                    break;
                case RIGHT:
                    randomPos = RandomPos.RIGHT;
                    break;
                case NOT_FOUND:
                    randomPos = RandomPos.NULL;
            }*/

            //If use camera toggle is on update randomPos based on location
            if(useCamera){
                if(randomPosByColorDetectionPipeline.location ==  RandomPosByColorDetectionPipeline.Location.NOT_FOUND){
                    randomPos = RandomPos.NULL;
                }
                else if(randomPosByColorDetectionPipeline.location ==  RandomPosByColorDetectionPipeline.Location.LEFT){
                    randomPos = RandomPos.LEFT;
                }
                else if (randomPosByColorDetectionPipeline.location ==  RandomPosByColorDetectionPipeline.Location.CENTER){
                    randomPos = RandomPos.CENTER;
                }
                else if (randomPosByColorDetectionPipeline.location ==  RandomPosByColorDetectionPipeline.Location.RIGHT){
                    randomPos = RandomPos.RIGHT;
                }
            }

            telemetry.addData("Color", startPosColor);
            telemetry.addData("Position", startPosPosition);
            telemetry.addData("Speed", speed);
            telemetry.addData("startSleep", startSleep);
            //telemetry.addData("Target Middle Pos.", targetMidPos);
            telemetry.addData("Park Pos.", parkPos);
            telemetry.addData("Combination", actionCombination);
            telemetry.addData("Random Pos.", randomPos);
            telemetry.addLine();
            telemetry.addData("Will Park", willPark);
            telemetry.addData("Will Board", willBoard);
            telemetry.addData("Will Spike", willSpike);
            telemetry.addLine();
            telemetry.addData("Left percentage", Math.round(randomPosByColorDetectionPipeline.leftValue * 100) + "%");
            telemetry.addData("Center percentage", Math.round(randomPosByColorDetectionPipeline.centerValue * 100) + "%");
            telemetry.addData("Right percentage", Math.round(randomPosByColorDetectionPipeline.rightValue * 100) + "%");
            telemetry.addData("Sensed Pos.", randomPosByColorDetectionPipeline.location);
            telemetry.addData("Use Camera?", useCamera);

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


        //Start command just came in

        //Stop the camera
        camera.stopStreaming();

        //Set modifier values
        switch (startPosColor){
            case RED:
                colorMod = -1;
                red = true;
                break;
            case BLUE:
                colorMod = 1;
                blue = true;
                break;
        }
        switch (startPosPosition){
            case FRONT:
                front = true;
                break;
            case BACK:
                back = true;
                break;
        }

        //2023-2024 Autonomous Main Code:

        //Wait
        sleep((long)((startSleep)*1000));

        //===Close claw===This might not be needed===
        //intake.closeClaw();

        //Blocks to run for different start positions
        if(red&&front){

            //1 of 6: Only Park
            if(actionCombination == ActionCombination.PARK_ONLY){
                //Move off the wall
                drivetrain.MoveForDis(4,speed);

                //To go to the middle of the backstage
                if(parkPos == ParkPos.MIDDLE){
                    //To move out to the middle
                    drivetrain.MoveForDis(51,speed);

                    //Retract arm to go under the gate
                    intake.runArm(intake.armMax);

                    //Turn and Move to the backstage
                    drivetrain.RotateForDegree(90*colorMod, speed-.25);
                    drivetrain.MoveForDis(96, speed);

                    //Move so not touching pixels hopefully
                    drivetrain.MoveForDis(-6,speed);

                    telemetry.addLine("Park complete");
                }

                //To go to the corner of the backstage
                else if(parkPos == ParkPos.CORNER){
                    //Retract arm to go under the gate
                    intake.runArm(intake.armMax);

                    //Turn and Move to the backstage
                    drivetrain.RotateForDegree(90*colorMod, speed-.25);
                    drivetrain.MoveForDis(96, speed);

                    //Move so not touching pixels hopefully
                    drivetrain.MoveForDis(-6,speed);

                    telemetry.addLine("Park complete");
                }
                else{
                    telemetry.addLine("Park Pos. Not set");
                }

                telemetry.addLine("Action: PARK_ONLY completed");
                telemetry.update();
                sleep(1000);
            }

            //2 of 6: Only Spike
            if(actionCombination == ActionCombination.SPIKE_ONLY){
                if(randomPos == RandomPos.LEFT){
                    //Move off the wall
                    drivetrain.MoveForDis(10,speed);

                    //Rotate for arm to place pixel
                    drivetrain.RotateForDegree(20, speed-.25);

                    //Run arm to place pixel on spike
                    intake.runArm(.10);
                    sleep(1000);

                    //Open claw, retract arm
                    intake.openClaw();
                    sleep(1000);
                    intake.closeClaw();
                    intake.runArm(.85);

                    telemetry.addLine("Pixel Placed on Spike");
                }
                else if(randomPos == RandomPos.CENTER){
                    //Move off the wall
                    drivetrain.MoveForDis(14,speed);

                    //Run arm to place pixel on spike
                    intake.runArm(.10);
                    sleep(1000);

                    //Open claw, retract arm
                    intake.openClaw();
                    sleep(1000);
                    intake.closeClaw();
                    intake.runArm(.85);

                    telemetry.addLine("Pixel Placed on Spike");
                }
                else if(randomPos == RandomPos.RIGHT){
                    //Move off the wall
                    drivetrain.MoveForDis(10,speed);

                    //Rotate for arm to place pixel
                    drivetrain.RotateForDegree(-30, speed-.25);

                    //Run arm to place pixel on spike
                    intake.runArm(.10);
                    sleep(1000);

                    //Open claw, retract arm
                    intake.openClaw();
                    sleep(1000);
                    intake.closeClaw();
                    intake.runArm(.85);

                    telemetry.addLine("Pixel Placed on Spike");
                }
                else if(randomPos == RandomPos.NULL){
                    telemetry.addLine("randomPos = NULL, can't do anything");
                }
                telemetry.update();

                //Realign with wall
                if(randomPos == RandomPos.LEFT){
                    drivetrain.RotateForDegree(-20, speed-.25);
                }
                else if (randomPos == RandomPos.RIGHT){
                    drivetrain.RotateForDegree(30, speed-.25);
                }

                telemetry.addLine("Action: SPIKE_ONLY completed");
                telemetry.update();
                sleep(1000);

            }

            //3 of 6: Park and Board*
            if(actionCombination == ActionCombination.PARK_BOARD){
                //Move off the wall
                drivetrain.MoveForDis(4,speed);

                //To go to the middle of the backstage
                if(parkPos == ParkPos.MIDDLE){
                    //To move out to the middle
                    drivetrain.MoveForDis(51,speed);

                    //Turn and Move to the backstage
                    drivetrain.RotateForDegree(90*colorMod, speed-.25);
                    drivetrain.MoveForDis(48, speed);

                    //Move so not touching pixels hopefully
                    drivetrain.MoveForDis(-6,speed);

                    telemetry.addLine("Park complete");
                }

                //To go to the corner of the backstage
                else if(parkPos == ParkPos.CORNER){
                    //Turn and Move to the backstage
                    drivetrain.RotateForDegree(90*colorMod, speed-.25);
                    drivetrain.MoveForDis(48, speed);

                    //Move so not touching pixels hopefully
                    drivetrain.MoveForDis(-6,speed);

                    telemetry.addLine("Park complete");
                }

                telemetry.addLine("Action: PARK_BOARD completed");
                telemetry.update();
                sleep(1000);
            }

            //4 of 6: Park and Spike*
            if(actionCombination == ActionCombination.PARK_SPIKE) {
                //What to do if the randomPos is Left
                if (randomPos == RandomPos.LEFT) {
                    //Move off the wall
                    drivetrain.MoveForDis(10, speed);

                    //Rotate for arm to place pixel
                    drivetrain.RotateForDegree(20, speed - .25);

                    //Run arm to place pixel on spike
                    intake.runArm(.10);
                    sleep(1000);

                    //Open claw, retract arm
                    intake.openClaw();
                    sleep(1000);
                    intake.closeClaw();
                    intake.runArm(.85);

                    telemetry.addLine("Pixel Placed on Spike");
                }
                //What to do if the randomPos is Center
                else if (randomPos == RandomPos.CENTER) {
                    //Move off the wall
                    drivetrain.MoveForDis(14, speed);

                    //Run arm to place pixel on spike
                    intake.runArm(.10);
                    sleep(1000);

                    //Open claw, retract arm
                    intake.openClaw();
                    sleep(1000);
                    intake.closeClaw();
                    intake.runArm(.85);

                    telemetry.addLine("Pixel Placed on Spike");
                }
                //What to do if the randomPos is Right
                else if (randomPos == RandomPos.RIGHT) {
                    //Move off the wall
                    drivetrain.MoveForDis(10, speed);

                    //Rotate for arm to place pixel
                    drivetrain.RotateForDegree(-30, speed - .25);

                    //Run arm to place pixel on spike
                    intake.runArm(.10);
                    sleep(1000);

                    //Open claw, retract arm
                    intake.openClaw();
                    sleep(1000);
                    intake.closeClaw();
                    intake.runArm(.85);

                    telemetry.addLine("Pixel Placed on Spike");
                }
                //To take an exception, **might be put to else OR put in front and if Null set to Left**
                else if (randomPos == RandomPos.NULL) {
                    telemetry.addLine("randomPos = NULL, can't do anything");
                }
                telemetry.update();

                //If randomPos is not NULL attempt to park
                if (randomPos != RandomPos.NULL) {
                    //Realign with wall
                    if (randomPos == RandomPos.LEFT) {
                        drivetrain.RotateForDegree(-20, speed - .25);
                    }
                    else if (randomPos == RandomPos.RIGHT) {
                        drivetrain.RotateForDegree(30, speed - .25);
                    }

                    //To go to the middle of the backstage when spike is in the way
                    if (parkPos == ParkPos.MIDDLE && randomPos == RandomPos.CENTER) {
                        telemetry.addLine("Can't park in Middle from front when randomPos is the center");
                        telemetry.update();
                    }
                    //To go to the middle of backstage if spike is not in the way
                    else if (parkPos == ParkPos.MIDDLE && randomPos != RandomPos.CENTER) {
                        //To move out to the middle
                        drivetrain.MoveForDis(51, speed);

                        //Retract arm to go under the gate
                        intake.runArm(intake.armMax);

                        //Turn and Move to the backstage
                        drivetrain.RotateForDegree(90 * colorMod, speed - .25);
                        drivetrain.MoveForDis(96, speed);

                        //Move so not touching pixels hopefully
                        drivetrain.MoveForDis(-6, speed);

                        telemetry.addLine("Park complete");
                    }
                    //To go to the corner of the backstage
                    else if (parkPos == ParkPos.CORNER) {
                            //Move back to be inline
                            drivetrain.MoveForDis(-7, speed - .2);

                            //Retract arm to go under the gate
                            intake.runArm(intake.armMax);

                            //Turn and Move to the backstage
                            drivetrain.RotateForDegree(90 * colorMod, speed - .25);
                            drivetrain.MoveForDis(96, speed);

                            telemetry.addLine("Park complete");
                        }
                        telemetry.update();
                    }
                else {
                    //****Should it park?****
                    telemetry.addLine("NULL can't move");
                }

                telemetry.addLine("Action: PARK_SPIKE completed");
                telemetry.update();
                sleep(1000);
            }

            //5 of 6: Park, Board, and Spike*
            if(actionCombination == ActionCombination.PARK_BOARD_SPIKE){
                //Move off the wall
                drivetrain.MoveForDis(4,speed);

                //***Place on spike***
                //***If not center and need not go to middle to park, realign and go to board***
                //***Place on board***
                //***Move away/park***

                telemetry.addLine("Action: PARK_BOARD_SPIKE completed");
                telemetry.update();
                sleep(1000);
            }

            //6 of 6: NONE
            if(actionCombination == ActionCombination.NONE){
                telemetry.addLine("Doing Nothing");
                telemetry.update();
                sleep(1000);
            }

            telemetry.update();
        }
        else if(red&&back){

            //1 of 6: Only Park
            if(actionCombination == ActionCombination.PARK_ONLY){
                //Move off the wall
                drivetrain.MoveForDis(4,speed);

                //To go to the middle of the backstage
                if(parkPos == ParkPos.MIDDLE){
                    //To move out to the middle
                    drivetrain.MoveForDis(51,speed);

                    //Turn and Move to the backstage
                    drivetrain.RotateForDegree(90*colorMod, speed-.25);
                    drivetrain.MoveForDis(48, speed);

                    //Move so not touching pixels hopefully
                    drivetrain.MoveForDis(-6,speed);

                    telemetry.addLine("Park complete");
                }

                //To go to the corner of the backstage
                else if(parkPos == ParkPos.CORNER){
                    //Turn and Move to the backstage
                    drivetrain.RotateForDegree(90*colorMod, speed-.25);
                    drivetrain.MoveForDis(48, speed);

                    //Move so not touching pixels hopefully
                    drivetrain.MoveForDis(-6,speed);

                    telemetry.addLine("Park complete");
                }
                else{
                    telemetry.addLine("Park Pos. Not set");
                }

                telemetry.addLine("Action: PARK_ONLY completed");
                telemetry.update();
                sleep(1000);
            }

            //2 of 6: Only Spike
            if(actionCombination == ActionCombination.SPIKE_ONLY){
                if(randomPos == RandomPos.LEFT){
                    //Move off the wall
                    drivetrain.MoveForDis(10,speed);

                    //Rotate for arm to place pixel
                    drivetrain.RotateForDegree(20, speed-.25);

                    //Run arm to place pixel on spike
                    intake.runArm(.10);
                    sleep(1000);

                    //Open claw, retract arm
                    intake.openClaw();
                    sleep(1000);
                    intake.closeClaw();
                    intake.runArm(.85);

                    telemetry.addLine("Pixel Placed on Spike");
                }
                else if(randomPos == RandomPos.CENTER){
                    //Move off the wall
                    drivetrain.MoveForDis(14,speed);

                    //Run arm to place pixel on spike
                    intake.runArm(.10);
                    sleep(1000);

                    //Open claw, retract arm
                    intake.openClaw();
                    sleep(1000);
                    intake.closeClaw();
                    intake.runArm(.85);

                    telemetry.addLine("Pixel Placed on Spike");
                }
                else if(randomPos == RandomPos.RIGHT){
                    //Move off the wall
                    drivetrain.MoveForDis(10,speed);

                    //Rotate for arm to place pixel
                    drivetrain.RotateForDegree(-30, speed-.25);

                    //Run arm to place pixel on spike
                    intake.runArm(.10);
                    sleep(1000);

                    //Open claw, retract arm
                    intake.openClaw();
                    sleep(1000);
                    intake.closeClaw();
                    intake.runArm(.85);

                    telemetry.addLine("Pixel Placed on Spike");
                }
                else if(randomPos == RandomPos.NULL){
                    telemetry.addLine("randomPos = NULL, can't do anything");
                }
                telemetry.update();

                //Realign with wall
                if(randomPos == RandomPos.LEFT){
                    drivetrain.RotateForDegree(-20, speed-.25);
                }
                else if (randomPos == RandomPos.RIGHT){
                    drivetrain.RotateForDegree(30, speed-.25);
                }

                telemetry.addLine("Action: SPIKE_ONLY completed");
                telemetry.update();
                sleep(1000);

            }

            //3 of 6: Park and Board*
            if(actionCombination == ActionCombination.PARK_BOARD){
                //Move off the wall
                drivetrain.MoveForDis(4,speed);

                //To go to the middle of the backstage
                if(parkPos == ParkPos.MIDDLE){
                    //To move out to the middle
                    drivetrain.MoveForDis(51,speed);

                    //Turn and Move to the backstage
                    drivetrain.RotateForDegree(90*colorMod, speed-.25);
                    drivetrain.MoveForDis(48, speed);

                    //Move so not touching pixels hopefully
                    drivetrain.MoveForDis(-6,speed);

                    telemetry.addLine("Park complete");
                }

                //To go to the corner of the backstage
                else if(parkPos == ParkPos.CORNER){
                    //Turn and Move to the backstage
                    drivetrain.RotateForDegree(90*colorMod, speed-.25);
                    drivetrain.MoveForDis(48, speed);

                    //Move so not touching pixels hopefully
                    drivetrain.MoveForDis(-6,speed);

                    telemetry.addLine("Park complete");
                }

                telemetry.addLine("Action: PARK_BOARD completed");
                telemetry.update();
                sleep(1000);
            }

            //4 of 6: Park and Spike
            if(actionCombination == ActionCombination.PARK_SPIKE){
                //What to do if the randomPos is Left
                if (randomPos == RandomPos.LEFT){
                    //Move off the wall
                    drivetrain.MoveForDis(10,speed);

                    //Rotate for arm to place pixel
                    drivetrain.RotateForDegree(20, speed-.25);

                    //Run arm to place pixel on spike
                    intake.runArm(.10);
                    sleep(1000);

                    //Open claw, retract arm
                    intake.openClaw();
                    sleep(1000);
                    intake.closeClaw();
                    intake.runArm(.85);

                    telemetry.addLine("Pixel Placed on Spike");
                }
                //What to do if the randomPos is Center
                else if (randomPos == RandomPos.CENTER){
                    //Move off the wall
                    drivetrain.MoveForDis(14,speed);

                    //Run arm to place pixel on spike
                    intake.runArm(.10);
                    sleep(1000);

                    //Open claw, retract arm
                    intake.openClaw();
                    sleep(1000);
                    intake.closeClaw();
                    intake.runArm(.85);

                    telemetry.addLine("Pixel Placed on Spike");
                }
                //What to do if the randomPos is Right
                else if (randomPos == RandomPos.RIGHT){
                    //Move off the wall
                    drivetrain.MoveForDis(10,speed);

                    //Rotate for arm to place pixel
                    drivetrain.RotateForDegree(-30, speed-.25);

                    //Run arm to place pixel on spike
                    intake.runArm(.10);
                    sleep(1000);

                    //Open claw, retract arm
                    intake.openClaw();
                    sleep(1000);
                    intake.closeClaw();
                    intake.runArm(.85);

                    telemetry.addLine("Pixel Placed on Spike");
                }
                //To take an exception, **might be put to else OR put in front and if Null set to Left**
                else if (randomPos == RandomPos.NULL){
                    telemetry.addLine("randomPos = NULL, can't do anything");
                }
                telemetry.update();

                //If randomPos is not NULL attempt to park
                if (randomPos != RandomPos.NULL){
                    //Realign with wall
                    if (randomPos == RandomPos.LEFT){
                        drivetrain.RotateForDegree(-20, speed-.25);
                    }
                    else if (randomPos == RandomPos.RIGHT){
                        drivetrain.RotateForDegree(30, speed-.25);
                    }

                    //To go to the middle of the backstage when spike is in the way
                    if (parkPos == ParkPos.MIDDLE && randomPos == RandomPos.CENTER){
                        //Turn and Move to the backstage
                        drivetrain.RotateForDegree(45*colorMod, speed-.25);
                        drivetrain.MoveForDis(60, speed);

                        //Move so not touching pixels hopefully
                        drivetrain.MoveForDis(-4,speed);

                        telemetry.addLine("Park complete");
                    }
                    //To go to the middle of backstage if spike is not in the way
                    if (parkPos == ParkPos.MIDDLE && randomPos != RandomPos.CENTER){
                        //To move out to the middle
                        drivetrain.MoveForDis(45,speed);

                        //Turn and Move to the backstage
                        drivetrain.RotateForDegree(90*colorMod, speed-.25);
                        drivetrain.MoveForDis(48, speed);

                        //Move so not touching pixels hopefully
                        drivetrain.MoveForDis(-6,speed);

                        telemetry.addLine("Park complete");
                    }
                    //To go to the corner of the backstage
                    else if (parkPos == ParkPos.CORNER){
                        //Move back to be inline
                        drivetrain.MoveForDis(-7,speed-.2);

                        //Turn and Move to the backstage
                        drivetrain.RotateForDegree(90*colorMod, speed-.25);
                        drivetrain.MoveForDis(42, speed);

                        telemetry.addLine("Park complete");
                    }
                    telemetry.update();
                }
                else {
                    //****Should it park?****
                    telemetry.addLine("NUll can't move");
                }

                telemetry.addLine("Action: PARK_SPIKE completed");
                telemetry.update();
                sleep(1000);
            }

            //5 of 6: Park, Board, and Spike*
            if(actionCombination == ActionCombination.PARK_BOARD_SPIKE){
                //Move off the wall
                drivetrain.MoveForDis(4,speed);

                //***Place on spike***
                //***If not center and need not go to middle to park, realign and go to board***
                //***Place on board***
                //***Move away/park***

                telemetry.addLine("Action: PARK_BOARD_SPIKE completed");
                telemetry.update();
                sleep(1000);
            }

            //6 of 6: NONE
            if(actionCombination == ActionCombination.NONE){
                telemetry.addLine("Doing Nothing");
                telemetry.update();
                sleep(1000);
            }

            telemetry.update();
        }
        else if(blue&&front){

            //1 of 6: Only Park
            if(actionCombination == ActionCombination.PARK_ONLY){
                //Move off the wall
                drivetrain.MoveForDis(5,speed);

                //To go to the middle of the backstage
                if(parkPos == ParkPos.MIDDLE){
                    //To move out to the middle
                    drivetrain.MoveForDis(51,speed);

                    //Retract arm to go under the gate
                    intake.runArm(intake.armMax);

                    //Turn and Move to the backstage
                    drivetrain.RotateForDegree(90*colorMod, speed-.25);
                    drivetrain.MoveForDis(96, speed);

                    //Move so not touching pixels hopefully
                    drivetrain.MoveForDis(-6,speed);

                    telemetry.addLine("Park complete");
                }

                //To go to the corner of the backstage
                else if(parkPos == ParkPos.CORNER){
                    //Retract arm to go under the gate
                    intake.runArm(intake.armMax);

                    //Turn and Move to the backstage
                    drivetrain.RotateForDegree(90*colorMod, speed-.25);
                    drivetrain.MoveForDis(96, speed);

                    //Move so not touching pixels hopefully
                    drivetrain.MoveForDis(-6,speed);

                    telemetry.addLine("Park complete");
                }
                else{
                    telemetry.addLine("Park Pos. Not set");
                }

                telemetry.addLine("Action: PARK_ONLY completed");
                telemetry.update();
                sleep(1000);
            }

            //2 of 6: Only Spike
            if(actionCombination == ActionCombination.SPIKE_ONLY){
                if(randomPos == RandomPos.LEFT){
                    //Move off the wall
                    drivetrain.MoveForDis(10,speed);

                    //Rotate for arm to place pixel
                    drivetrain.RotateForDegree(20, speed-.25);

                    //Run arm to place pixel on spike
                    intake.runArm(.10);
                    sleep(1000);

                    //Open claw, retract arm
                    intake.openClaw();
                    sleep(1000);
                    intake.closeClaw();
                    intake.runArm(.85);

                    telemetry.addLine("Pixel Placed on Spike");
                }
                else if(randomPos == RandomPos.CENTER){
                    //Move off the wall
                    drivetrain.MoveForDis(14,speed);

                    //Run arm to place pixel on spike
                    intake.runArm(.10);
                    sleep(1000);

                    //Open claw, retract arm
                    intake.openClaw();
                    sleep(1000);
                    intake.closeClaw();
                    intake.runArm(.85);

                    telemetry.addLine("Pixel Placed on Spike");
                }
                else if(randomPos == RandomPos.RIGHT){
                    //Move off the wall
                    drivetrain.MoveForDis(10,speed);

                    //Rotate for arm to place pixel
                    drivetrain.RotateForDegree(-30, speed-.25);

                    //Run arm to place pixel on spike
                    intake.runArm(.10);
                    sleep(1000);

                    //Open claw, retract arm
                    intake.openClaw();
                    sleep(1000);
                    intake.closeClaw();
                    intake.runArm(.85);

                    telemetry.addLine("Pixel Placed on Spike");
                }
                else if(randomPos == RandomPos.NULL){
                    telemetry.addLine("randomPos = NULL, can't do anything");
                }
                telemetry.update();

                //Realign with wall
                if (randomPos == RandomPos.LEFT){
                    drivetrain.RotateForDegree(-20, speed-.25);
                }
                else if (randomPos == RandomPos.RIGHT){
                    drivetrain.RotateForDegree(30, speed-.25);
                }

                telemetry.addLine("Action: SPIKE_ONLY completed");
                telemetry.update();
                sleep(1000);

            }

            //3 of 6: Park and Board*
            if(actionCombination == ActionCombination.PARK_BOARD){
                //Move off the wall
                drivetrain.MoveForDis(4,speed);

                //To go to the middle of the backstage
                if(parkPos == ParkPos.MIDDLE){
                    //To move out to the middle
                    drivetrain.MoveForDis(51,speed);

                    //Turn and Move to the backstage
                    drivetrain.RotateForDegree(90*colorMod, speed-.25);
                    drivetrain.MoveForDis(48, speed);

                    //Move so not touching pixels hopefully
                    drivetrain.MoveForDis(-6,speed);

                    telemetry.addLine("Park complete");
                }

                //To go to the corner of the backstage
                else if(parkPos == ParkPos.CORNER){
                    //Turn and Move to the backstage
                    drivetrain.RotateForDegree(90*colorMod, speed-.25);
                    drivetrain.MoveForDis(48, speed);

                    //Move so not touching pixels hopefully
                    drivetrain.MoveForDis(-6,speed);

                    telemetry.addLine("Park complete");
                }

                telemetry.addLine("Action: PARK_BOARD completed");
                telemetry.update();
                sleep(1000);
            }

            //4 of 6: Park and Spike*
            if(actionCombination == ActionCombination.PARK_SPIKE){
                //What to do if the randomPos is Left
                if(randomPos == RandomPos.LEFT){
                    //Move off the wall
                    drivetrain.MoveForDis(10,speed);

                    //Rotate for arm to place pixel
                    drivetrain.RotateForDegree(20, speed-.25);

                    //Run arm to place pixel on spike
                    intake.runArm(.10);
                    sleep(1000);

                    //Open claw, retract arm
                    intake.openClaw();
                    sleep(1000);
                    intake.closeClaw();
                    intake.runArm(.85);

                    telemetry.addLine("Pixel Placed on Spike");
                }
                //What to do if the randomPos is Center
                else if(randomPos == RandomPos.CENTER){
                    //Move off the wall
                    drivetrain.MoveForDis(14,speed);

                    //Run arm to place pixel on spike
                    intake.runArm(.10);
                    sleep(1000);

                    //Open claw, retract arm
                    intake.openClaw();
                    sleep(1000);
                    intake.closeClaw();
                    intake.runArm(.85);

                    telemetry.addLine("Pixel Placed on Spike");
                }
                //What to do if the randomPos is Right
                else if(randomPos == RandomPos.RIGHT){
                    //Move off the wall
                    drivetrain.MoveForDis(10,speed);

                    //Rotate for arm to place pixel
                    drivetrain.RotateForDegree(-30, speed-.25);

                    //Run arm to place pixel on spike
                    intake.runArm(.10);
                    sleep(1000);

                    //Open claw, retract arm
                    intake.openClaw();
                    sleep(1000);
                    intake.closeClaw();
                    intake.runArm(.85);

                    telemetry.addLine("Pixel Placed on Spike");
                }
                //To take an exception, **might be put to else OR put in front and if Null set to Left**
                else if(randomPos == RandomPos.NULL){
                    telemetry.addLine("randomPos = NULL, can't do anything");
                }
                telemetry.update();

                //If randomPos is not NULL attempt to park
                if(randomPos != RandomPos.NULL){
                    //Realign with wall
                    if(randomPos == RandomPos.LEFT){
                        drivetrain.RotateForDegree(-20, speed-.25);
                    }
                    else if (randomPos == RandomPos.RIGHT){
                        drivetrain.RotateForDegree(30, speed-.25);
                    }

                    //To go to the middle of the backstage when spike is in the way
                    if(parkPos == ParkPos.MIDDLE && randomPos == RandomPos.CENTER){
                        telemetry.addLine("Can't park in Middle from front when randomPos is the center");
                        telemetry.update();
                    }
                    //To go to the middle of backstage if spike is not in the way
                    if(parkPos == ParkPos.MIDDLE && randomPos != RandomPos.CENTER){
                        //To move out to the middle
                        drivetrain.MoveForDis(51,speed);

                        //Retract arm to go under the gate
                        intake.runArm(intake.armMax);

                        //Turn and Move to the backstage
                        drivetrain.RotateForDegree(90*colorMod, speed-.25);
                        drivetrain.MoveForDis(96, speed);

                        //Move so not touching pixels hopefully
                        drivetrain.MoveForDis(-6,speed);

                        telemetry.addLine("Park complete");
                    }
                    //To go to the corner of the backstage
                    else if(parkPos == ParkPos.CORNER){
                        //Move back to be inline
                        drivetrain.MoveForDis(-7,speed-.2);

                        //Retract arm to go under the gate
                        intake.runArm(intake.armMax);

                        //Turn and Move to the backstage
                        drivetrain.RotateForDegree(90*colorMod, speed-.25);
                        drivetrain.MoveForDis(96, speed);

                        telemetry.addLine("Park complete");
                    }
                    telemetry.update();
                }
                else {
                    //****Should it park?****
                    telemetry.addLine("NUll can't move");
                }


                telemetry.addLine("Action: PARK_SPIKE completed");
                telemetry.update();
                sleep(1000);
            }

            //5 of 6: Park, Board, and Spike*
            if(actionCombination == ActionCombination.PARK_BOARD_SPIKE){
                //Move off the wall
                drivetrain.MoveForDis(4,speed);

                //***Place on spike***
                //***If not center and need not go to middle to park, realign and go to board***
                //***Place on board***
                //***Move away/park***

                telemetry.addLine("Action: PARK_BOARD_SPIKE completed");
                telemetry.update();
                sleep(1000);
            }

            //6 of 6: NONE
            if(actionCombination == ActionCombination.NONE){
                telemetry.addLine("Doing Nothing");
                telemetry.update();
                sleep(1000);
            }

            telemetry.update();
        }
        else if(blue&&back){

            //1 of 6: Only Park
            if(actionCombination == ActionCombination.PARK_ONLY){
                //Move off the wall
                drivetrain.MoveForDis(4,speed);

                //To go to the middle of the backstage
                if(parkPos == ParkPos.MIDDLE){
                    //To move out to the middle
                    drivetrain.MoveForDis(51,speed);

                    //Turn and Move to the backstage
                    drivetrain.RotateForDegree(90*colorMod, speed-.25);
                    drivetrain.MoveForDis(48, speed);

                    //Move so not touching pixels hopefully
                    drivetrain.MoveForDis(-6,speed);

                    telemetry.addLine("Park complete");
                }

                //To go to the corner of the backstage
                else if(parkPos == ParkPos.CORNER){
                    //Turn and Move to the backstage
                    drivetrain.RotateForDegree(90*colorMod, speed-.25);
                    drivetrain.MoveForDis(48, speed);

                    //Move so not touching pixels hopefully
                    drivetrain.MoveForDis(-6,speed);

                    telemetry.addLine("Park complete");
                }
                else{
                    telemetry.addLine("Park Pos. Not set");
                }

                telemetry.addLine("Action: PARK_ONLY completed");
                telemetry.update();
                sleep(1000);
            }

            /*2 of 8: Only Board
            if(actionCombination == actionCombination.BOARD_ONLY){
                //***Go to board***
                //***Based on random pos place pixel on board***
                //***Move out of the way***
            }*/

            //2 of 6: Only Spike
            if(actionCombination == ActionCombination.SPIKE_ONLY){
                if(randomPos == RandomPos.LEFT){
                    //Move off the wall
                    drivetrain.MoveForDis(10,speed);

                    //Rotate for arm to place pixel
                    drivetrain.RotateForDegree(20, speed-.25);

                    //Run arm to place pixel on spike
                    intake.runArm(.10);
                    sleep(1000);

                    //Open claw, retract arm
                    intake.openClaw();
                    sleep(1000);
                    intake.closeClaw();
                    intake.runArm(.85);

                    telemetry.addLine("Pixel Placed on Spike");
                }
                else if(randomPos == RandomPos.CENTER){
                    //Move off the wall
                    drivetrain.MoveForDis(14,speed);

                    //Run arm to place pixel on spike
                    intake.runArm(.10);
                    sleep(1000);

                    //Open claw, retract arm
                    intake.openClaw();
                    sleep(1000);
                    intake.closeClaw();
                    intake.runArm(.85);

                    telemetry.addLine("Pixel Placed on Spike");
                }
                else if(randomPos == RandomPos.RIGHT){
                    //Move off the wall
                    drivetrain.MoveForDis(10,speed);

                    //Rotate for arm to place pixel
                    drivetrain.RotateForDegree(-30, speed-.25);

                    //Run arm to place pixel on spike
                    intake.runArm(.10);
                    sleep(1000);

                    //Open claw, retract arm
                    intake.openClaw();
                    sleep(1000);
                    intake.closeClaw();
                    intake.runArm(.85);

                    telemetry.addLine("Pixel Placed on Spike");
                }
                else if(randomPos == RandomPos.NULL){
                    telemetry.addLine("randomPos = NULL, can't do anything");
                }
                telemetry.update();

                //Realign with wall
                if(randomPos == RandomPos.LEFT){
                    drivetrain.RotateForDegree(-20, speed-.25);
                }
                else if (randomPos == RandomPos.RIGHT){
                    drivetrain.RotateForDegree(30, speed-.25);
                }

                telemetry.addLine("Action: SPIKE_ONLY completed");
                telemetry.update();
                sleep(1000);

            }

            //3 of 6: Park and Board*
            if(actionCombination == ActionCombination.PARK_BOARD){
                //Move off the wall
                drivetrain.MoveForDis(4,speed);

                //To go to the middle of the backstage
                if(parkPos == ParkPos.MIDDLE){
                    //To move out to the middle
                    drivetrain.MoveForDis(51,speed);

                    //Turn and Move to the backstage
                    drivetrain.RotateForDegree(90*colorMod, speed-.25);
                    drivetrain.MoveForDis(48, speed);

                    //Move so not touching pixels hopefully
                    drivetrain.MoveForDis(-6,speed);

                    telemetry.addLine("Park complete");
                }

                //To go to the corner of the backstage
                else if(parkPos == ParkPos.CORNER){
                    //Turn and Move to the backstage
                    drivetrain.RotateForDegree(90*colorMod, speed-.25);
                    drivetrain.MoveForDis(48, speed);

                    //Move so not touching pixels hopefully
                    drivetrain.MoveForDis(-6,speed);

                    telemetry.addLine("Park complete");
                }

                telemetry.addLine("Action: PARK_BOARD completed");
                telemetry.update();
                sleep(1000);
            }

            /*5 of 8: Spike and Board
            if(actionCombination == actionCombination.SPIKE_BOARD){
                //***Place on Spike***
                //***Move to Board***
                //***Place on Board***
                //***Move out of the way/park***
            }*/

            //4 of 6: Park and Spike
            if(actionCombination == ActionCombination.PARK_SPIKE){
                //What to do if the randomPos is Left
                if(randomPos == RandomPos.LEFT){
                    //Move off the wall
                    drivetrain.MoveForDis(10,speed);

                    //Rotate for arm to place pixel
                    drivetrain.RotateForDegree(20, speed-.25);

                    //Run arm to place pixel on spike
                    intake.runArm(.10);
                    sleep(1000);

                    //Open claw, retract arm
                    intake.openClaw();
                    sleep(1000);
                    intake.closeClaw();
                    intake.runArm(.85);

                    telemetry.addLine("Pixel Placed on Spike");
                }
                //What to do if the randomPos is Center
                else if(randomPos == RandomPos.CENTER){
                    //Move off the wall
                    drivetrain.MoveForDis(14,speed);

                    //Run arm to place pixel on spike
                    intake.runArm(.10);
                    sleep(1000);

                    //Open claw, retract arm
                    intake.openClaw();
                    sleep(1000);
                    intake.closeClaw();
                    intake.runArm(.85);

                    telemetry.addLine("Pixel Placed on Spike");
                }
                //What to do if the randomPos is Right
                else if(randomPos == RandomPos.RIGHT){
                    //Move off the wall
                    drivetrain.MoveForDis(10,speed);

                    //Rotate for arm to place pixel
                    drivetrain.RotateForDegree(-30, speed-.25);

                    //Run arm to place pixel on spike
                    intake.runArm(.10);
                    sleep(1000);

                    //Open claw, retract arm
                    intake.openClaw();
                    sleep(1000);
                    intake.closeClaw();
                    intake.runArm(.85);

                    telemetry.addLine("Pixel Placed on Spike");
                }
                //To take an exception, **might be put to else OR put in front and if Null set to Left**
                else if(randomPos == RandomPos.NULL){
                    telemetry.addLine("randomPos = NULL, can't do anything");
                }
                telemetry.update();

                //Attempt to park if randomPos is not Null
                if(randomPos != RandomPos.NULL){
                    //Realign with wall
                    if(randomPos == RandomPos.LEFT){
                        drivetrain.RotateForDegree(-20, speed-.25);
                    }
                    else if (randomPos == RandomPos.RIGHT){
                        drivetrain.RotateForDegree(30, speed-.25);
                    }

                    //To go to the middle of the backstage when spike is in the way
                    if(parkPos == ParkPos.MIDDLE && randomPos == RandomPos.CENTER){
                        //Turn and Move to the backstage
                        drivetrain.RotateForDegree(45*colorMod, speed-.25);
                        drivetrain.MoveForDis(60, speed);

                        //Move so not touching pixels hopefully
                        drivetrain.MoveForDis(-4,speed);

                        telemetry.addLine("Park complete");
                    }
                    //To go to the middle of the backstage when spike is not in the way
                    if(parkPos == ParkPos.MIDDLE && randomPos != RandomPos.CENTER){
                        //To move out to the middle
                        drivetrain.MoveForDis(45,speed);

                        //Turn and Move to the backstage
                        drivetrain.RotateForDegree(90*colorMod, speed-.25);
                        drivetrain.MoveForDis(48, speed);

                        //Move so not touching pixels hopefully
                        drivetrain.MoveForDis(-6,speed);

                        telemetry.addLine("Park complete");
                    }
                    //To go to the corner of the backstage
                    else if(parkPos == ParkPos.CORNER){
                        //Move back to be inline
                        drivetrain.MoveForDis(-7,speed-.2);

                        //Turn and Move to the backstage
                        drivetrain.RotateForDegree(90*colorMod, speed-.25);
                        drivetrain.MoveForDis(42, speed);

                        telemetry.addLine("Park complete");
                    }
                    telemetry.update();
                }
                else {
                    //****Should it park?****
                    telemetry.addLine("NUll can't move");
                }

                telemetry.addLine("Action: PARK_SPIKE completed");
                telemetry.update();
                sleep(1000);
            }

            //5 of 6: Park, Board, and Spike*
            if(actionCombination == ActionCombination.PARK_BOARD_SPIKE){
                //Move off the wall
                drivetrain.MoveForDis(4,speed);

                //***Place on spike***
                //***If not center and need not go to middle to park, realign and go to board***
                //***Place on board***
                //***Move away/park***

                telemetry.addLine("Action: PARK_BOARD_SPIKE completed");
                telemetry.update();
                sleep(1000);
            }

            //6 of 6: NONE
            if(actionCombination == ActionCombination.NONE){
                telemetry.addLine("Doing Nothing");
                telemetry.update();
                sleep(1000);
            }

            telemetry.update();
        }

    /*void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));

    }*/

    }
}