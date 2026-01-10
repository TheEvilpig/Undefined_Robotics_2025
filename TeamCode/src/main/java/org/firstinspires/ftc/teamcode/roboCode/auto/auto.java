package org.firstinspires.ftc.teamcode.roboCode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@Disabled
@Autonomous(name="simple auto",group = "Autonomous")
public class auto extends LinearOpMode{
    private DcMotorEx frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "lf");
        backLeftDrive = hardwareMap.get(DcMotorEx.class, "lb");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "rf");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "rb");
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        waitForStart();

        if(opModeIsActive()){
            frontLeftDrive.setPower(0.7);
            backRightDrive.setPower(0.7);
            frontRightDrive.setPower(0.7);
            backLeftDrive.setPower(0.7);
            sleep(2000);
            frontLeftDrive.setPower(0);
            backRightDrive.setPower(0);
            frontRightDrive.setPower(0);
            backLeftDrive.setPower(0);
        }
    }
}
