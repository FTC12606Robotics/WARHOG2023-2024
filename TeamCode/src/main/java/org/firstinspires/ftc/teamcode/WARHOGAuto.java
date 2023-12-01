package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name="WARHOGAuto", group="")
public class WARHOGAuto extends LinearOpMode {

    public WARHOGAuto() throws InterruptedException {}

    private StartPosColor startPosColor = StartPosColor.RED;
    private enum StartPosColor {RED, BLUE}
    private StartPosPosition startPosPosition = StartPosPosition.FRONT;
    private enum StartPosPosition {FRONT, BACK}
    private ParkPos parkPos = ParkPos.CORNER;
    private enum ParkPos {NO, CORNER, MIDDLE} //For where to park if at all
    private RandomPos randomPos = RandomPos.NULL;
    private enum RandomPos {NULL, LEFT, CENTER, RIGHT} //For what position the randomization is in
    private ActionCombination actionCombination = ActionCombination.PARK_ONLY;
    private enum ActionCombination {PARK_ONLY/*, BOARD_ONLY*/, SPIKE_ONLY, PARK_BOARD, NONE/*, SPIKE_BOARD*/, PARK_SPIKE, PARK_BOARD_SPIKE}

    //OpenCvCamera camera;
    //AprilTagDetectionPipeline aprilTagDetectionPipeline;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    static final double FEET_PER_METER = 3.28084;

    int colorMod = 0;
    //int posMod = 0;
    //int cycles = 2;

    boolean front=false, back=false, red=false, blue=false; //Bools to set position

    //targetMidPos is for the legacy code
    boolean targetMidPos = false; //To set whether to park in the corner of the backstage or middle of it
    boolean willPark = false; //for interior code use for if the robot will park
    boolean willSpike = false; //for interior code use for if the robot will place a pixel on a spike
    boolean willBoard = false; //for interior code use for if the robot will place a pixel on the backdrop

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
                /*if(targetMidPos){
                    targetMidPos = false;
                }
                else if(!targetMidPos){
                    targetMidPos = true;
                }*/
                if(parkPos == parkPos.MIDDLE){
                    parkPos = parkPos.CORNER;
                }
                else if (parkPos == parkPos.CORNER){
                    parkPos = parkPos.NO;
                }
                else if (parkPos == parkPos.NO){
                    parkPos = parkPos.MIDDLE;
                }
            }

            //Go through different combinations of things to do and set bools
            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper){
                if(actionCombination == actionCombination.PARK_ONLY){
                    actionCombination = actionCombination.SPIKE_ONLY;
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
                else if(actionCombination == actionCombination.SPIKE_ONLY){
                    actionCombination = actionCombination.PARK_BOARD;
                    willPark = true;
                    willBoard = true;
                    willSpike = false;
                }
                else if(actionCombination == actionCombination.PARK_BOARD){
                    actionCombination = actionCombination.PARK_SPIKE;
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
                else if(actionCombination == actionCombination.PARK_SPIKE){
                    actionCombination = actionCombination.PARK_BOARD_SPIKE;
                    willPark = true;
                    willBoard = true;
                    willSpike = true;
                }
                else if(actionCombination == actionCombination.PARK_BOARD_SPIKE){
                    actionCombination = actionCombination.NONE;
                    willPark = false;
                    willBoard = false;
                    willSpike = false;
                }
                else if(actionCombination == actionCombination.NONE){
                    actionCombination = actionCombination.PARK_ONLY;
                    willPark = true;
                    willBoard = false;
                    willSpike = false;
                }
            }

            telemetry.addData("Color", startPosColor);
            telemetry.addData("Position", startPosPosition);
            telemetry.addData("Speed", speed);
            telemetry.addData("startSleep", startSleep);
            //telemetry.addData("Target Middle Pos.", targetMidPos);
            telemetry.addData("Park Pos.", parkPos);
            telemetry.addData("Combination", actionCombination);
            telemetry.addLine();
            telemetry.addData("Will Park", willPark);
            telemetry.addData("Will Board", willBoard);
            telemetry.addData("Will Spike", willSpike);
            telemetry.addData("Random Pos.", randomPos);

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

            //Find and set which random pos. the pixel/gameobject is in.


            telemetry.update();
            sleep(20);
        }


        //Start command just came in

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
                //posMod = -1;
                front = true;
                break;
            case BACK:
                //posMod = 1;
                back = true;
                break;
        }

        /*
        //2023-2024 Autonomous Legacy Main Code:

        //Wait
        sleep((long)((startSleep)*1000));

        //Blocks to run for different start positions
        if(red&&front){
            //Wait and then move off the wall
            //sleep((long)((startSleep)*1000));
            drivetrain.MoveForDis(4,speed);

            //Check if we are going to the backstage middle
            if(targetMidPos){
                drivetrain.MoveForDis(51,speed);
            }

            //Retract arm to go under gate
            intake.runArm(intake.armMax);

            //Turn and Move to the backstage
            drivetrain.RotateForDegree(90, speed-.25);
            drivetrain.MoveForDis(96, speed);

            //Move so not touching pixels hopefully
            drivetrain.MoveForDis(-6,speed);

            telemetry.addLine("Park complete");
            telemetry.update();
        }
        else if(red&&back){
            //Wait and then move off the wall
            //sleep((long)(startSleep*1000));
            drivetrain.MoveForDis(4,speed);

            //Check if we are going to the backstage middle
            if(targetMidPos){
                drivetrain.MoveForDis(51,speed);
            }

            //Turn and Move to the backstage
            drivetrain.RotateForDegree(90, speed-.25);
            drivetrain.MoveForDis(48, speed);

            //Move so not touching pixels hopefully
            drivetrain.MoveForDis(-6,speed);

            telemetry.addLine("Park complete");
            telemetry.update();
        }
        else if(blue&&front){
            //Wait and then move off the wall
            //sleep((long)((startSleep)*1000));
            drivetrain.MoveForDis(4,speed);

            //Check if we are going to the backstage middle
            if(targetMidPos){
                drivetrain.MoveForDis(51,speed);
            }

            //Retract arm to go under the gate
            intake.runArm(intake.armMax);

            //Turn and Move to the backstage
            drivetrain.RotateForDegree(-90, speed-.25);
            drivetrain.MoveForDis(96, speed);

            //Move so not touching pixels hopefully
            drivetrain.MoveForDis(-6,speed);

            telemetry.addLine("Park complete");
            telemetry.update();
        }
        else if(blue&&back){
            //Wait and then move off the wall
            //sleep((long)((startSleep)*1000));
            drivetrain.MoveForDis(4,speed);

            //Check if we are going to the backstage middle
            if(targetMidPos){
                drivetrain.MoveForDis(51,speed);
            }

            //Retract arm to go under the gate
            intake.runArm(intake.armMax);

            //Turn and Move to the backstage
            drivetrain.RotateForDegree(-90, speed-.25);
            drivetrain.MoveForDis(96, speed);

            //Move so not touching pixels hopefully
            drivetrain.MoveForDis(-6,speed);

            telemetry.addLine("Park complete");
            telemetry.update();
        }
        */


        //2023-2024 Autonomous Main Code:

        //Wait
        sleep((long)((startSleep)*1000));

        //Close claw
        intake.closeClaw();

        //Blocks to run for different start positions
        if(red&&front){
            //Move off the wall
            drivetrain.MoveForDis(4,speed);

            //Check if we are going to the backstage middle
            if(parkPos == parkPos.MIDDLE){
                drivetrain.MoveForDis(51,speed);
            }

            //Retract arm to go under gate
            intake.runArm(intake.armMax);

            //Turn and Move to the backstage
            drivetrain.RotateForDegree(90*colorMod, speed-.25);
            drivetrain.MoveForDis(96, speed);

            //Move so not touching pixels hopefully
            drivetrain.MoveForDis(-6,speed);

            telemetry.addLine("Park complete");
            telemetry.update();
        }
        else if(red&&back){
            //Move off the wall
            drivetrain.MoveForDis(4,speed);

            //Check if we are going to the backstage middle
            if(parkPos == parkPos.MIDDLE){
                drivetrain.MoveForDis(51,speed);
            }

            //Turn and Move to the backstage
            drivetrain.RotateForDegree(90*colorMod, speed-.25);
            drivetrain.MoveForDis(48, speed);

            //Move so not touching pixels hopefully
            drivetrain.MoveForDis(-6,speed);

            telemetry.addLine("Park complete");
            telemetry.update();
        }
        else if(blue&&front){
            //Move off the wall
            drivetrain.MoveForDis(4,speed);

            //Check if we are going to the backstage middle
            if(parkPos == parkPos.MIDDLE){
                drivetrain.MoveForDis(51,speed);
            }

            //Retract arm to go under the gate
            intake.runArm(intake.armMax);

            //Turn and Move to the backstage
            drivetrain.RotateForDegree(90*colorMod, speed-.25);
            drivetrain.MoveForDis(96, speed);

            //Move so not touching pixels hopefully
            drivetrain.MoveForDis(-6,speed);

            telemetry.addLine("Park complete");
            telemetry.update();
        }
        else if(blue&&back){
            //Move off the wall
            drivetrain.MoveForDis(4,speed);

            //1 of 6: Only Park
            if(actionCombination == actionCombination.PARK_ONLY){
                //To go to the middle of the backstage
                if(parkPos == parkPos.MIDDLE){
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
                else if(parkPos == parkPos.CORNER){
                    //Turn and Move to the backstage
                    drivetrain.RotateForDegree(90*colorMod, speed-.25);
                    drivetrain.MoveForDis(48, speed);

                    //Move so not touching pixels hopefully
                    drivetrain.MoveForDis(-6,speed);

                    telemetry.addLine("Park complete");
                }

                telemetry.addLine("Action: PARK_ONLY completed");
            }

            /*2 of 8: Only Board
            if(actionCombination == actionCombination.BOARD_ONLY){
                //***Go to board***
                //***Based on random pos place pixel on board***
                //***Move out of the way***
            }*/

            //2 of 6: Only Spike*
            if(actionCombination == actionCombination.SPIKE_ONLY){
                //***Based on random pos move to where robot can place down spike***

                //***Run arm to place spike***
                intake.runArm(.10);
                sleep(2000);

                //***Open claw, retract arm***
                intake.openClaw();
                sleep(1000);
                intake.closeClaw();
                intake.runArm(intake.armMax);

                telemetry.addLine("Pixel Placed on Spike");
                //***Realign with wall***
                telemetry.addLine("Action: SPIKE_ONLY completed");
                telemetry.update();
                sleep(3000);

            }

            //3 of 6: Park and Board*
            if(actionCombination == actionCombination.PARK_BOARD){
                //To go to the middle of the backstage
                if(parkPos == parkPos.MIDDLE){
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
                else if(parkPos == parkPos.CORNER){
                    //Turn and Move to the backstage
                    drivetrain.RotateForDegree(90*colorMod, speed-.25);
                    drivetrain.MoveForDis(48, speed);

                    //Move so not touching pixels hopefully
                    drivetrain.MoveForDis(-6,speed);

                    telemetry.addLine("Park complete");
                }

                telemetry.addLine("Action: PARK_BOARD completed");
            }

            /*5 of 8: Spike and Board
            if(actionCombination == actionCombination.SPIKE_BOARD){
                //***Place on Spike***
                //***Move to Board***
                //***Place on Board***
                //***Move out of the way/park***
            }*/

            //4 of 6: Park and Spike*
            if(actionCombination == actionCombination.PARK_SPIKE){
                //***Based on random pos move to where robot can place down spike***

                //++++CODE inaction 2++++
                //***Run arm to place spike***
                //***Open claw, retract arm***

                telemetry.addLine("Pixel Placed on Spike");

                //***Realign with wall***

                //***Remember to make sure going to the middle does not affect spikes***
                //To go to the middle of the backstage
                if(parkPos == parkPos.MIDDLE){
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
                else if(parkPos == parkPos.CORNER){
                    //Turn and Move to the backstage
                    drivetrain.RotateForDegree(90*colorMod, speed-.25);
                    drivetrain.MoveForDis(48, speed);

                    //Move so not touching pixels hopefully
                    drivetrain.MoveForDis(-6,speed);

                    telemetry.addLine("Park complete");
                }

                telemetry.addLine("Action: PARK_SPIKE completed");
            }

            //5 of 6: Park, Board, and Spike*
            if(actionCombination == actionCombination.PARK_BOARD_SPIKE){
                //***Place on spike***
                //***If not center and need not go to middle to park, realign and go to board***
                //***Place on board***
                //***Move away/park***

                telemetry.addLine("Action: PARK_BOARD_SPIKE completed");
            }

            //6 of 6: NONE
            if(actionCombination == actionCombination.NONE){
                telemetry.addLine("Doing Nothing");
            }


            //To not park at all
           /* else if(!willPark && parkPos == parkPos.NO){
                telemetry.addLine("Not Parking");
            }*/

            telemetry.update();
        }


        /*
        //2022-2023 Code

        // drive to pole and raise slide
        drivetrain.MoveForDis(52.5, speed);
        if(cycles>-1) {
            drivetrain.rotateToPosition(-43 * posMod - (posMod+1), speed - .25);
            drivetrain.MoveForDis(-.75, speed);
            telemetry.addLine("just before slides");
            telemetry.update();
            outtake.setHeight(Outtake.Height.HIGH);
            telemetry.addLine("height added");
            telemetry.update();
            sleep(200);
            outtake.setHeight(1500);
            outtake.openClaw();
            outtake.setHeight(Outtake.Height.GROUND);
            //outtake.run(-1);
            //drivetrain.MoveForDis(-.5, speed);
            //drivetrain.MoveForDis(-.75, speed);

            // turn to cone stack
            //drivetrain.MoveForDis(1, speed);

            telemetry.addLine("Stage 1 complete");
        }
        else{
            cycles = 0;
        }


        for(int i = 0; i < cycles; i++) {
            //drivetrain.RotateForDegree(-45 * posMod, speed);
            drivetrain.rotateToPosition(-84 * posMod-2*(posMod-1), speed-.45-.25*(posMod-1));
            intake.runArm(.16-.0325*i);
            sleep(400);

            // move backward toward cone stack
            drivetrain.MoveForDis(-11.5, speed*.75);

            // take another cone
            intake.closeClaw();
            intake.changeWristMode(Intake.WristMode.INDEPENDENT);
            sleep(500);
            intake.runArm(.4);
            sleep(500);
            intake.changeWristMode(Intake.WristMode.MATCHED);
            intake.runArm(Intake.Height.RETRACTED);

            // turn back
            drivetrain.MoveForDis(11.5, speed);
            //sleep(250);
            intake.openClaw();
            sleep(500);
            outtake.closeClaw();
            //drivetrain.RotateForDegree(45 * posMod, speed);
            drivetrain.rotateToPosition(-45 * posMod, speed*.75);
            intake.runArm(Intake.Height.DRIVESIZING);
            //drivetrain.MoveForDis(.5, speed);
            //drivetrain.MoveForDis(.75, 0.2);
            //sleep(250);
            outtake.setHeight(Outtake.Height.HIGH);
            telemetry.addLine("height added");
            telemetry.update();
            sleep(200);
            outtake.setHeight(1500);
            outtake.openClaw();
            outtake.setHeight(Outtake.Height.GROUND);
            //outtake.run(-1);
            if (i < cycles - 1) {
                //drivetrain.MoveForDis(-.5, speed);
            }
        }
        telemetry.addLine("Stage 2 complete");
        telemetry.update();

        // park
        intake.runArm(Intake.Height.RETRACTED);
        //drivetrain.RotateForDegree(-45 * posMod, speed);
        if(tagOfInterest == null || tagOfInterest.id == MIDDLE){

        }else if((tagOfInterest.id-2)*posMod==1){
            drivetrain.rotateToPosition(-90 * posMod, speed-.25);
            drivetrain.MoveForDis(-21, speed);

        }else{
            drivetrain.rotateToPosition(-90 * posMod, speed-.25);
            drivetrain.MoveForDis(22, speed);
        }

        //drivetrain.RotateForDegree(90*posMod, speed);
        drivetrain.rotateToPosition(0, speed-.25);
        if(tagOfInterest != null && (tagOfInterest.id-2)*posMod==1) {
            drivetrain.SideMoveForDis(2.5*posMod, speed);
        }
        drivetrain.MoveForDis(-12, speed);
        intake.runArm(Intake.Height.RETRACTED);
        telemetry.addLine("Stage 3 complete");
        telemetry.update();
    }*/

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

}