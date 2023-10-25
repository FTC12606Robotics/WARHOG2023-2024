package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import static java.lang.Math.*;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import static java.lang.String.*;
import java.util.ArrayList;

@TeleOp(name="WARHOGTeleOp", group="")
public class WARHOGTeleOp extends LinearOpMode {
    public WARHOGTeleOp() throws InterruptedException {}

    @Override
    public void runOpMode() throws InterruptedException {

        //set up classes
        Drivetrain drivetrain = new Drivetrain(hardwareMap, telemetry);
        Intake intake = new Intake(hardwareMap, telemetry);
        Outtake outtake = new Outtake(hardwareMap, telemetry);
        WaitForLoops waitForLoops = new WaitForLoops();
        //For drone Launch
        DroneLaunch droneLaunch = new DroneLaunch(hardwareMap, telemetry);

        //set up variables
        double joyx, joyy, joyz, gas, basespeed, armpos, wristmod, offset, slideMovement,
                maxIncrease, armposChange, intakeArmSpeed=.03, modAngle;
        boolean autoEjectMode = false;
        boolean autoIntakeMode = false;
        boolean pauseToResetMaxIncrease = false;
        boolean stationary = false;
        boolean outtakeGround, outtakeLow, outtakeMedium, outtakeHigh, toggleOuttakeClaw = false,
                centricityToggle, resetDriveAngle, autoEjectToggle, autoIntakeToggle,
                stationaryToggle, toggleIntakeClaw, oneDriver = false, oneDriverToggle,
                extendIntakeArm = false, retractIntakeArm = false, uprightIntakeArm = false, sizingIntakeArm = false,
                intakeCone=false, wristFixed = false, wristFixedToggle = false, isOuttakeAtTarget,
                outtakeClawMoveIntake = false;
        //For drone launch
        boolean launchDrone;

        int leftConeStack = 5, rightConeStack = 5;

        offset = 0;
        Drivetrain.Centricity centricity = Drivetrain.Centricity.FIELD;

        basespeed = .4;
        armpos = intake.runArm(Intake.Height.UPRIGHT);

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        droneLaunch.ArmDrone();

        while (!isStarted() && !isStopRequested()) {
            outtake.openClaw();
            armpos = intake.runArm(Intake.Height.UPRIGHT);
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

            if(currentGamepad1.dpad_left && !previousGamepad1.dpad_left){
                offset-=90;
            }
            if(currentGamepad1.dpad_right && !previousGamepad1.dpad_right){
                offset+=90;
            }
            if (offset==360){offset=0;}
            if (offset==-90){offset=270;}

            telemetry.addData("Angle Offset", offset);
            telemetry.update();
        }

        drivetrain.setAngleOffset(offset);
        oneDriver = false;
        autoEjectMode = false;
        autoIntakeMode = false;

        while(opModeIsActive()){
            //set up inputs
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
            telemetry.addData("angle", drivetrain.getIMUData()/PI*180);

            isOuttakeAtTarget = outtake.update();

    //set up inputs

            //inputs that toggle the modes
            centricityToggle = currentGamepad1.dpad_down && !previousGamepad1.dpad_down; //change whether the drive is bot or field centric
            autoEjectToggle = currentGamepad2.start && !previousGamepad2.start;
            autoIntakeToggle = currentGamepad2.back && !previousGamepad2.back;
            stationaryToggle = currentGamepad1.back && !previousGamepad1.back;
            oneDriverToggle = currentGamepad1.start && !previousGamepad1.start;
            wristFixedToggle = currentGamepad2.left_trigger>.2 && !(previousGamepad2.left_trigger>.2);

            //change the modes based on the inputs
            if(wristFixedToggle) {
                if(wristFixed) {
                    wristFixed = false;
                }
                else{
                    wristFixed = true;
                }
            }
            if(oneDriverToggle){
                if(oneDriver){
                    oneDriver=false;
                }
                else{
                    oneDriver=true;
                }
            }
            if(stationaryToggle){
                if(stationary){
                    stationary=false;
                }
                else{
                    stationary=true;
                }
            }
            if(autoEjectToggle){
                if(autoEjectMode){
                    autoEjectMode=false;
                }
                else{
                    autoEjectMode=true;
                }
            }
            if(autoIntakeToggle){
                if(autoIntakeMode){
                    autoIntakeMode=false;
                }
                else{
                    autoIntakeMode=true;
                }
            }



            resetDriveAngle = currentGamepad1.dpad_up; //use when the robot is facing away from you


            telemetry.addData("Stationary", stationary);
            //set up operator commands based on whether oneDriver mode is on
            if(oneDriver){
                outtakeGround = false;
                outtakeLow = false;
                outtakeMedium = false;
                outtakeHigh = false;
                maxIncrease = 0;
                //if outtake claw is closed, or if slides are up, you control the outtake
                if(!outtake.isClawOpen() || outtake.showSlideValue()>0){
                    sizingIntakeArm = true;
                    if(currentGamepad1.left_bumper){
                        slideMovement = 1;
                    }
                    else if(currentGamepad1.left_trigger>.2){
                        slideMovement = -1;
                    }
                    else{
                        slideMovement = 0;
                    }
                    //toggle outtake claw
                    toggleOuttakeClaw = currentGamepad1.right_bumper && !previousGamepad1.right_bumper;

                    armposChange=0;
                    toggleIntakeClaw=false;
                }
                else{
                    sizingIntakeArm = false;
                    //left shoulder buttons always control arm if outtake claw is closed and down
                    if(currentGamepad1.left_bumper){
                        armposChange = -intakeArmSpeed;
                    }
                    else if(currentGamepad1.left_trigger>.2){
                        armposChange = intakeArmSpeed;
                    }
                    else{
                        armposChange = 0;
                    }
                    //if intake claw is closed, claw button controls intake claw
                    if(!intake.isClawOpen()){
                        toggleIntakeClaw = currentGamepad1.right_bumper && !previousGamepad1.right_bumper;
                        toggleOuttakeClaw = false;
                    }
                    else{
                        if(intake.getArmPos()>.4){
                            toggleOuttakeClaw = currentGamepad1.right_bumper && !previousGamepad1.right_bumper;
                            toggleIntakeClaw = false;
                        }
                        else{
                            toggleIntakeClaw = currentGamepad1.right_bumper && !previousGamepad1.right_bumper;
                            toggleOuttakeClaw = false;
                        }
                    }

                    slideMovement = 0;
                }
            }
            else {
                //code to switch between field centric and bot centric drive
                if(centricityToggle){
                    if(centricity==Drivetrain.Centricity.BOT){
                        centricity = Drivetrain.Centricity.FIELD;
                    }
                    else{
                        centricity = Drivetrain.Centricity.BOT;
                    }
                }

                armposChange = currentGamepad2.left_stick_y*intakeArmSpeed;
                toggleIntakeClaw = currentGamepad2.left_bumper && !previousGamepad2.left_bumper;
                if(autoIntakeMode){
                    intakeCone = currentGamepad2.dpad_down;
                    //if(toggleIntakeClaw){
                        //waitForLoops.addWaitEvent("Raise Intake Arm", 20);
                    //}
                }
                else {
                    retractIntakeArm = currentGamepad2.dpad_down;
                }
                sizingIntakeArm = currentGamepad2.dpad_right || (outtakeClawMoveIntake&&intake.getArmPos()>.7);
                uprightIntakeArm = currentGamepad2.dpad_left;
                extendIntakeArm = currentGamepad2.dpad_up && !previousGamepad2.dpad_up;

                //set up slide commands based on whether stationary mode is on
                if (!stationary) {
                    slideMovement = -currentGamepad2.right_stick_y;
                    outtakeGround = currentGamepad2.a;
                    outtakeLow = currentGamepad2.x;
                    outtakeMedium = currentGamepad2.b;
                    outtakeHigh = currentGamepad2.y;
                    toggleOuttakeClaw = (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) || (currentGamepad2.right_trigger>.2 && !(previousGamepad2.right_trigger>.2));
                    outtakeClawMoveIntake = currentGamepad2.right_bumper && !previousGamepad2.right_bumper;
                    maxIncrease = currentGamepad2.right_trigger * 100;
                } else {
                    if(currentGamepad1.left_bumper){
                        slideMovement = 1;
                    }
                    else if(currentGamepad1.left_trigger>.2){
                        slideMovement = -1;
                    }
                    else{
                        slideMovement = 0;
                    }
                    telemetry.addData("gamepad 1 left trigger", currentGamepad1.left_trigger);
                    outtakeGround = currentGamepad1.a;
                    outtakeLow = currentGamepad1.x;
                    outtakeMedium = currentGamepad1.b;
                    outtakeHigh = currentGamepad1.y;
                    toggleOuttakeClaw = (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) || (currentGamepad1.right_trigger>.2 && !(previousGamepad1.right_trigger>.2));
                    outtakeClawMoveIntake = currentGamepad1.right_bumper && !previousGamepad1.right_bumper;
                    maxIncrease = currentGamepad1.left_trigger * 100;
                }
            }


            //set up vectors
            joyx = currentGamepad1.left_stick_x;
            joyy = -currentGamepad1.left_stick_y;
            joyz = currentGamepad1.right_stick_x;
            gas = currentGamepad1.right_trigger*(1-basespeed);

            //print vectors
            telemetry.addData("y", joyy);
            telemetry.addData("x", joyx);
            telemetry.addData("z", joyz);



            //turn off wrist fixed if arm is over a certain threshold
            if(toggleIntakeClaw&&intake.isClawOpen()&&intake.getArmPos()>.01){
                wristFixed=true;
            }
            if(toggleIntakeClaw&&!intake.isClawOpen()){
                wristFixed=false;
            }
            if(intake.getArmPos()>.95){
                wristFixed=false;
            }

            if(wristFixed){
                intake.changeWristMode(Intake.WristMode.INDEPENDENT);
            }
            else{
                intake.changeWristMode(Intake.WristMode.MATCHED);
            }



            //set and print motor powers
            double[] motorPowers = drivetrain.driveVectors(centricity, joyx, joyy, joyz, basespeed+gas);
            for (double line:motorPowers){
                telemetry.addLine( Double.toString(line) );
            }

            //reset the angle
            if(resetDriveAngle){
                drivetrain.resetAngle();
            }

            //move arm
            armpos += armposChange;
            if(armpos<0){armpos=0;}
            if(armpos>1){armpos=1;}
            //defined positions
            if(retractIntakeArm){
                armpos = intake.runArm(Intake.Height.RETRACTED);
            }
            modAngle = (drivetrain.getIMUData()/PI*180)%360;
            telemetry.addData("mod angle", modAngle);
            telemetry.addData("left cone stack", leftConeStack);
            telemetry.addData("right cone stack", rightConeStack);
            if(extendIntakeArm){
                if(modAngle>45 && modAngle<135){
                    armpos = .15-.0375*(5-leftConeStack);
                    leftConeStack -= 1;
                    if(leftConeStack<1){
                        leftConeStack = 5;
                    }
                }
                else if(modAngle<-45 && modAngle>-135){
                    armpos = .15-.0375*(5-rightConeStack);
                    rightConeStack -= 1;
                    if(rightConeStack<1){
                        rightConeStack = 5;
                    }
                }else {
                    armpos = intake.runArm(Intake.Height.EXTENDED);
                }
            }
            if(uprightIntakeArm){
                armpos = intake.runArm(Intake.Height.UPRIGHT);
            }
            if(sizingIntakeArm){
                armpos = intake.runArm(Intake.Height.DRIVESIZING);
            }
            if(intakeCone){
                intake.intakeCone();
            }

            //move the arm, modifying the wrist's position if right trigger is pressed
            wristmod = 0; //(currentGamepad2.left_trigger-.2)*.625;
            if(wristmod>0){
                intake.runArm(armpos, wristmod);
                telemetry.addData("Wrist Mod", wristmod);
            }
            else {
                intake.runArm(armpos);
            }
            telemetry.addData("Arm Position", armpos);

            //open/close the claw
            if(toggleIntakeClaw){
                intake.toggleClaw();
            }



            //open/close the outtake claw
            if(toggleOuttakeClaw){
                outtake.toggleClaw();
                /*if(!stationary) {
                    armpos = intake.runArm(Intake.Height.UPRIGHT);
                }*/
                telemetry.addLine("Toggle OuttakeClaw");
            }

            //Set Up input to launch drone
            launchDrone = gamepad2.right_stick_button;

            //Launch Drone
            if(launchDrone){
                droneLaunch.LaunchDrone();
            }

            //increase slide maximum
            if(outtake.showSlideValue()>1600 && maxIncrease>0) {
                if (!pauseToResetMaxIncrease) {
                    outtake.increaseMax(currentGamepad2.right_trigger * 50, currentGamepad2.right_stick_button && !previousGamepad2.right_stick_button);
                    if (currentGamepad2.right_stick_button && !previousGamepad2.right_stick_button) {
                        pauseToResetMaxIncrease = true;
                    }
                } else {
                    if (currentGamepad2.right_trigger == 0) {
                        pauseToResetMaxIncrease = false;
                    }
                }
            }

            //move the outtake slides up and down
            if(!outtake.isSlideGoingToPosition()) {
                outtake.run(slideMovement);
                telemetry.addLine("moving using stick");
            }

            if(outtakeGround){
                outtake.setHeightWithoutWaiting(Outtake.Height.GROUND);
                telemetry.addLine("A");
            }
            if(outtakeLow){
                outtake.setHeightWithoutWaiting(Outtake.Height.LOW);
                telemetry.addLine("X");
            }
            if(outtakeMedium){
                outtake.setHeightWithoutWaiting(Outtake.Height.MEDIUM);
                telemetry.addLine("B");
            }
            if(outtakeHigh){
                outtake.setHeightWithoutWaiting(Outtake.Height.HIGH);
                telemetry.addLine("Y");
            }

            telemetry.addData("slide max", outtake.getMax());

            if(waitForLoops.checkEvent("Raise Intake Arm")){
                intake.runArm(Intake.Height.DRIVESIZING);
                outtake.setTarget(0);
            }
            if(isOuttakeAtTarget&&outtake.getTarget()==0){
                outtake.closeClaw();
            }

            //end step
            telemetry.update();
            waitForLoops.update();
        }

    }

    private class WaitForLoops{
        int loopsSoFar = 0;

        ArrayList<String> names = new ArrayList<String>();
        ArrayList<Integer> waitTimes = new ArrayList<Integer>();

        public void update(){
            loopsSoFar+=1;
        }

        public void addWaitEvent(String name, int loops){
            if(names.contains(name)){
                return;
            }
            names.add(name);
            waitTimes.add(loopsSoFar+loops);
        }

        public boolean checkEvent(String name){
            if(!names.contains(name)){
                return false;
            }
            boolean isEventTriggered = waitTimes.get(names.indexOf(name)) >= loopsSoFar;
            if(isEventTriggered){
                waitTimes.remove(names.indexOf(name));
                names.remove(names.indexOf(name));
            }
            return isEventTriggered;
        }
    }
}