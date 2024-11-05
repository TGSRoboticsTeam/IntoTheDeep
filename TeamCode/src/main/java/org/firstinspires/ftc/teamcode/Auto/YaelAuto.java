package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "YaelAuto", group = "YaelAuto")
//@Disabled
public class YaelAuto extends LinearOpMode {

    GoBildaPinpointDriver odo;
    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    @Override
    public void runOpMode() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Waits until driver presses PLAY
        waitForStart();
        runtime.reset();

        // Run auto
        driveForward(1000, 0.1);
        rotateDegrees(90, 0.5);
        // telemetry.addData("Testing...");
        telemetry.update();



    }

    public double distanceTo(double x, double y) {
        double deltaX = odo.getPosX() - x;
        double deltaY = odo.getPosY() - y;
        double offset = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

        return offset;
    }

    public void drive(double forward, double sideways, double rotation) {
        // Drive
        double axial = -forward;
        double lateral = sideways;
        double yaw = rotation;

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
        double rightBack = axial + lateral - yaw;
        double leftBack = axial - lateral + yaw;
        double leftFront = axial + lateral + yaw;
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

        // Associates buttons/joysticks to motors/servos:
        // Wheels
        leftFrontDrive.setPower(leftFront);
        leftBackDrive.setPower(leftBack);
        rightFrontDrive.setPower(rightFront);
        rightBackDrive.setPower(rightBack);
    }

    public void driveForward(double millimeters, double power) {
        double originalX = odo.getPosX();
        double originalY = odo.getPosY();
        while (distanceTo(originalX, originalY) < millimeters) {
            drive(power, 0, 0);
            telemetry.addData("Distance Traveled", distanceTo(originalX, originalY));
            telemetry.update();
        }
    }

    public void rotateDegrees(double degrees, double power) {
        double originalRot = (odo.getHeading() * 180) / Math.PI;
        while (Math.abs(originalRot - odo.getHeading()) < degrees) {
            drive(0, 0, power);
            telemetry.addData("Rotation", originalRot - odo.getHeading());
            telemetry.update();
        }
    }
        // Show the elapsed game time and wheel power.
        //telemetry.addData("Status", "Run Time: " + runtime.toString());
        // telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontDrive, rightFrontDrive);
        //telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackDrive, rightBackDrive);
        //telemetry.update();
}

