package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
//import Objects.*;

@Autonomous(name = "Yael Auto", group = "Yael Auto")
public class YaelAuto extends LinearOpMode {
        Drive drive;
        @Override
        public void runOpMode() {

            drive = new Drive();
            waitForStart();
            // Need this so that the code will stay initialized until you hit play on the phone
           // while (!isStarted()) {
               // pass;
            //}

            while (opModeIsActive()) {
                drive.driveForward(3);

            }
        }
}


