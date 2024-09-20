package org.firstinspires.ftc.teamcode.TeleOp;

/*import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;*/
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Yael Drive", group = "Yael Drive")
@Disabled
public class YaelDriveJr extends LinearOpMode{
    @Override
    public void runOpMode() {
        /////////////// MOTORS //////////////////
        // Motor Setup
        DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        // Sets the motor direction
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // Makes the motors stop moving when they receive an input of 0
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //////////// SERVOS ////////////
        Servo grabber = hardwareMap.get(Servo.class, "grabber_servo");

        /////////// OTHER /////////////
        // Set up FtcDashboard telemetry
        /*FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();*/

        double changeInSpeed = 0.2;

        // Need this so that the code will stay initialized until you hit play on the phone
        while (!isStarted()) {

        }

        while (opModeIsActive()) {
            /* Define control variables */
            // Claw
            boolean grab = gamepad2.b;

            // Drive
            double axial   = -gamepad1.left_stick_y;
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            boolean slowDown = gamepad1.left_bumper;

            if (axial <= 0.1 && axial >= -0.1) {
                axial = 0;
            }

            if (lateral <= 0.1 && lateral >= -0.1) {
                lateral = 0;
            }

            if (yaw <= 0.1 && yaw >= -0.1) {
                yaw = 0;
            }

            // Gives the joystick commands purpose, "mecanum" wheel stuff or whatever
            double rightFront = axial - lateral - yaw;
            double rightBack  = axial + lateral - yaw;
            double leftBack   = axial - lateral + yaw;
            double leftFront  = axial + lateral + yaw;
            double max;

            max = Math.max(Math.abs(leftFront), Math.abs(rightFront));
            max = Math.max(max, Math.abs(leftBack));
            max = Math.max(max, Math.abs(rightBack));

            if (max > 1.0) {
                leftFront  /= max;
                rightFront /= max;
                leftBack   /= max;
                rightBack  /= max;
            }

            if (slowDown){
                leftFront  *= changeInSpeed;
                rightFront *= changeInSpeed;
                leftBack   *= changeInSpeed;
                rightBack  *= changeInSpeed;
            }

            // Grabbing
            if (grab) {
                if (grabber.getPosition() == 1) {
                    grabber.setPosition(0);
                }else{
                    grabber.setPosition(1);
                }
            }

            // Associates buttons/joysticks to motors/servos:
            // Wheels
            leftFrontDrive.setPower(leftFront);
            leftBackDrive.setPower(leftBack);
            rightFrontDrive.setPower(rightFront);
            rightBackDrive.setPower(rightBack);

            //dashboardTelemetry.addData("Left Slide Pos: ", leftLinearSlide.getCurrentPosition());
            //dashboardTelemetry.addData("Right Slide Pos: ", rightLinearSlide.getCurrentPosition());
            //dashboardTelemetry.update();
        }
    }
}
