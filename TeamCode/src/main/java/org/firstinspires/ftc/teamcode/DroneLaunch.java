package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DroneLaunch {

    private Servo launcher;
    private Telemetry telemetry;
    boolean launched = false;

    double armedPos = 0.0;
    double launchedPos = 0.7;

    DroneLaunch(HardwareMap hardwareMap, Telemetry telemetry){
        launcher = hardwareMap.get(Servo.class, "launcher");
    }

    public void ArmDrone() {

        launcher.setPosition(armedPos);

        /*if (!launched) {
            //Keep Drone Launcher Armed
            launcher.setPosition(armedPos);
        }*/
    }

    //Call to launch the drone
    public void LaunchDrone(){
        //Launch Drone
        launched = true;
        launcher.setPosition(launchedPos);
        telemetry.addLine("Drone Launched");
        telemetry.update();
    }
}
