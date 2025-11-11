package org.firstinspires.ftc.teamcode.roboCode.tele;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@TeleOp(name="servo_test", group="Linear OpMode")
public class servo_test extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime;

    private Servo b1, b2;



    Orientation angles;
    @Override
    public void runOpMode() {


        b1 = hardwareMap.get(Servo.class, "ls");
        b2 = hardwareMap.get(Servo.class, "rs");


        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (gamepad1.a) {
                b1.setPosition(b1.getPosition() + 0.002);
            }

            if (gamepad1.b) {
                b1.setPosition(b1.getPosition() - 0.002);
            }

            if (gamepad1.dpad_up) {
                b2.setPosition(b2.getPosition() + 0.002);
            }

            if (gamepad1.dpad_down) {
                b2.setPosition(b2.getPosition() - 0.002);
            }

            telemetry.addData("b1:", "%f", b1.getPosition());
            telemetry.addData("b2:", "%f", b2.getPosition());

            telemetry.update();
        }

    }


}








