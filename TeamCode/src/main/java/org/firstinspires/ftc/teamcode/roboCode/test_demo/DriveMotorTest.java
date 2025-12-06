package org.firstinspires.ftc.teamcode.roboCode.test_demo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Motor Test: Drive Motors", group="Testing")
public class DriveMotorTest extends LinearOpMode {

    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;

    @Override
    public void runOpMode() {
        // Initialize hardware
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "lf");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "lb");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rf");
        backRightDrive  = hardwareMap.get(DcMotor.class, "rb");

        // Reverse left side to match your drivetrain configuration
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("Drive Motor Test Initialized");
        telemetry.addLine("Press (X)=LF, (Y)=RF, (A)=LB, (B)=RB");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Stop all motors by default
            double lfPower = 0, lbPower = 0, rfPower = 0, rbPower = 0;

            // Each button runs one motor forward
            if (gamepad1.x) lfPower = 0.5;   // front left
            if (gamepad1.y) rfPower = 0.5;   // front right
            if (gamepad1.a) lbPower = 0.5;   // back left
            if (gamepad1.b) rbPower = 0.5;   // back right

            frontLeftDrive.setPower(lfPower);
            backLeftDrive.setPower(lbPower);
            frontRightDrive.setPower(rfPower);
            backRightDrive.setPower(rbPower);

            // Telemetry feedback
            telemetry.addData("LF (X)", lfPower);
            telemetry.addData("RF (Y)", rfPower);
            telemetry.addData("LB (A)", lbPower);
            telemetry.addData("RB (B)", rbPower);
            telemetry.update();
        }
    }
}
