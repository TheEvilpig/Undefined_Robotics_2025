package org.firstinspires.ftc.teamcode.roboCode.tele;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import kotlin.math.UMathKt;

@TeleOp(name="Testing", group="Linear OpMode")
public class testing extends LinearOpMode {
    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    //Brakes
    private Servo carol;
    private double pos;
    @Override

    public void runOpMode() {
        //chassis
        telemetry.addData("Status", "Initialized");

        //</editor-fold>
        carol = hardwareMap.get(Servo.class,"serv");
        carol.setDirection(Servo.Direction.REVERSE);
        waitForStart();

        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            pos = gamepad1.right_trigger;
            pos = -gamepad1.left_trigger;
            carol.setPosition(pos);
        }

    }
}