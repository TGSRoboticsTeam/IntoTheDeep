package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
//@Disabled
public class Drive extends LinearOpMode{
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    // speed constants
    private double speed;
    private double turnSpeed;
    ElapsedTime timer = new ElapsedTime();
    public Drive(){
        runOpMode();
    }
    public void runOpMode() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        // Sets the motor direction
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Makes the motors stop moving when they receive an input of 0
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

         timer = new ElapsedTime();
    }

    public void drive(int vertical,int horizontal){
            // Drive
            double axial = -vertical;
            double lateral = horizontal;

            if (axial <= 0.1 && axial >= -0.1) {
                axial = 0;
            }

            if (lateral <= 0.1 && lateral >= -0.1) {
                lateral = 0;
            }

            // Gives the joystick commands purpose, "mecanum" wheel stuff or whatever
            double rightFront = axial - lateral;
            double rightBack = axial + lateral;
            double leftBack = axial - lateral;
            double leftFront = axial + lateral;
            double max;

            max = Math.max(Math.abs(leftFront), Math.abs(rightFront));
            max = Math.max(max, Math.abs(leftBack));
            max = Math.max(max, Math.abs(rightBack));

            if (max > 1.0) {
                leftFront /= max;
                rightFront /= max;
                leftBack /= max;
                rightBack /= max;
            }

            // Wheels
            leftFrontDrive.setPower(leftFront);
            leftBackDrive.setPower(leftBack);
            rightFrontDrive.setPower(rightFront);
            rightBackDrive.setPower(rightBack);
        }

        public void rotate(int direction){
            // Drive
            double yaw = direction;

            if (yaw <= 0.1 && yaw >= -0.1) {
                yaw = 0;
            }

            // Gives the joystick commands purpose, "mecanum" wheel stuff or whatever
            double rightFront = -yaw;
            double rightBack = -yaw;
            double leftBack = yaw;
            double leftFront = yaw;
            double max;

            max = Math.max(Math.abs(leftFront), Math.abs(rightFront));
            max = Math.max(max, Math.abs(leftBack));
            max = Math.max(max, Math.abs(rightBack));

            if (max > 1.0) {
                leftFront /= max;
                rightFront /= max;
                leftBack /= max;
                rightBack /= max;
            }

            // Wheels
            leftFrontDrive.setPower(leftFront);
            leftBackDrive.setPower(leftBack);
            rightFrontDrive.setPower(rightFront);
            rightBackDrive.setPower(rightBack);
        }
        public void rotateTime(double seconds, int dir){
            timer.reset();
            while (timer.seconds() <= seconds) {
                rotate(dir);
            }
        }

        public void rotateLeft() {
            rotateTime(1, -1);
        }

        public void rotateRight() {
            rotateTime(1, 1);
        }

        public void driveForward(double seconds){
            timer.reset();
            while (timer.seconds() < seconds) {
                drive(1, 0);
            }
        }

        public void driveBackward(double seconds){
            timer.reset();
            while (timer.seconds() < seconds) {
                drive(-1, 0);
            }
        }

        public void driveLeft(double seconds){
            timer.reset();
            while (timer.seconds() < seconds) {
                drive(0, -1);
            }
        }

        public void driveRight(double seconds){
            timer.reset();
            while(timer.seconds() < seconds) {
                drive(0, 1);
            }
        }
    }

