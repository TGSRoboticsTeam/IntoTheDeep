package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ServoImpl;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

//@Disabled
@TeleOp(name = "YaelDriveJr", group = "YaelDriveJr")

public class YaelDriveJr extends LinearOpMode {
    @Override
    public void runOpMode() {
        //GamepadEx gamepadEx = new GamepadEx(gamepad2); // probably not needed...
        DcMotor linearSlide = hardwareMap.get(DcMotor.class, "linear_slide");
        linearSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Makes the motors output their rotation
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Servo armServo = hardwareMap.get(Servo.class, "arm_servo");
        Servo wristServo = hardwareMap.get(Servo.class, "grabber_servo");
        Servo grabber = hardwareMap.get(Servo.class, "grabber");

        // Motor Setup
        DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        // Sets the motor direction
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Makes the motors stop moving when they receive an input of 0
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set up FtcDashboard telemetry
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        double changeInSpeed = 0.2;
        boolean justGrabbed = false;
        boolean justMovedArm = false;

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT, // Change to left if doesn't work
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        while (!isStarted()) {

        }

        while (opModeIsActive()) {
            // Define joystick controls
            double moveSlide = -gamepad2.left_stick_y;
            //float moveSlide = gamepad2.left_trigger - gamepad2.right_trigger;

            //boolean toggleArm = gamepad2.right_bumper;
            //boolean toggleGrabber = gamepad2.left_bumper;

            boolean toggleArm = gamepad1.x;
            boolean toggleGrabber = gamepad1.y;
            boolean closeGrabber = gamepad1.a;
            boolean openGrabber = gamepad1.b;

            boolean armPos1 = gamepad2.dpad_right;
            boolean armPos2 = gamepad2.dpad_up;
            boolean armPos3 = gamepad2.dpad_left;
            boolean armPos4 = gamepad2.dpad_down;

            // Drive
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            boolean slowDown = gamepad1.left_bumper;

            if (gamepad1.dpad_up) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bots rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;
            //telemetry.addData("fl, bl,fr,br: ", "%.2f %.2f %.2f %.2f",frontLeftPower,backLeftPower,frontRightPower,backRightPower);

            if (slowDown) {
                frontLeftPower *= changeInSpeed;
                frontRightPower *= changeInSpeed;
                backLeftPower *= changeInSpeed;
                backRightPower *= changeInSpeed;
            }

            double roundDown = 0.1;
            if (Math.abs(frontLeftPower) <= roundDown) {
                frontLeftPower = 0;
            }
            if (Math.abs(frontRightPower) <= roundDown) {
                frontRightPower = 0;
            }
            if (Math.abs(backLeftPower) <= roundDown) {
                backLeftPower = 0;
            }
            if (Math.abs(backRightPower) <= roundDown) {
                backRightPower = 0;
            }

            leftFrontDrive.setPower(frontLeftPower);
            leftBackDrive.setPower(backLeftPower);
            rightFrontDrive.setPower(frontRightPower);
            rightBackDrive.setPower(backRightPower);

            //////////////// OTHER COMPONENTS //////////////////
            telemetry.addData("moveSlide var before", moveSlide);
            if (linearSlide.getCurrentPosition() >= 2210 && moveSlide > 0) {
                moveSlide = 0;
            } else if (linearSlide.getCurrentPosition() <= 2 && moveSlide < 0) {
                moveSlide = 0;
            } else if (linearSlide.getCurrentPosition() <= 1105 && moveSlide == 0) {
                moveSlide = 0.1;
            }
            double linearSlowDown = 0.75;
            linearSlide.setPower(moveSlide * linearSlowDown);

            telemetry.addData("moveSlide var after", moveSlide);
            telemetry.addData("Linear Slide", linearSlide.getCurrentPosition());
            telemetry.addData("Arm Servo", armServo.getPosition());
            telemetry.addData("Wrist Servo", wristServo.getPosition());
            telemetry.addData("Grabber Servo", grabber.getPosition());
            //*
            if (armPos1 || armPos2 || armPos3 || armPos4) {
                if (armPos1) {
                    armServo.setPosition(-1);
                    wristServo.setPosition(1);
                }else if (armPos2) {
                    armServo.setPosition(-0.5);
                    wristServo.setPosition(0.5);
                }else if (armPos3) {
                    armServo.setPosition(0);
                    wristServo.setPosition(0);
                }else if (armPos4) {
                    armServo.setPosition(0.5);
                    wristServo.setPosition(-0.5);
                }
            }else{
                // Grabbing
                double grabbingPos = 1;
                if (toggleGrabber && !justGrabbed) {
                    justGrabbed = true;
                    if (grabber.getPosition() == grabbingPos) {
                        grabber.setPosition(0);
                    } else {
                        grabber.setPosition(grabbingPos);
                    }
                } else if (openGrabber) {
                    grabber.setPosition(grabbingPos);
                } else if (closeGrabber) {
                    grabber.setPosition(0);
                } else {
                    justGrabbed = false;
                }

                // Arm rotate
                double armPos = 1;
                if (toggleArm && !justMovedArm) {
                    justMovedArm = true;
                    if (armServo.getPosition() == armPos) {
                        armServo.setPosition(0);
                    } else {
                        armServo.setPosition(armPos);
                    }
                } else {
                    justMovedArm = false;
                }
                //*/
            }

            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            telemetry.addData("Pitch (X)", "%.2f", orientation.getPitch(AngleUnit.DEGREES));
            telemetry.addData("Roll (Y)", "%.2f", orientation.getRoll(AngleUnit.DEGREES));
            telemetry.addData("Yaw (Z)", "%.2f", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.update();
        }
    }
}

