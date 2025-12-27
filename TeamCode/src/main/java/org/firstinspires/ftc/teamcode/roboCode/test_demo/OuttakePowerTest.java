package org.firstinspires.ftc.teamcode.roboCode.test_demo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.HConst;

@TeleOp(name="Outtake Power Test", group="Linear OpMode")
public class OuttakePowerTest extends LinearOpMode {

    private DcMotorEx outtake1, outtake2;

    private ElapsedTime runtime = new ElapsedTime();

    double vel = 0;

    double target = 0;

    @Override
    public void runOpMode() {

        // Initialize hardware
        outtake1 = hardwareMap.get(DcMotorEx.class, HConst.OUTTAKE1);
        outtake2 = hardwareMap.get(DcMotorEx.class, HConst.OUTTAKE2);

        // Set directions if needed
        outtake1.setDirection(DcMotorEx.Direction.FORWARD);
        outtake2.setDirection(DcMotorEx.Direction.FORWARD);
        outtake1.setDirection(HConst.OUTTAKE1_DIR);
        outtake2.setDirection(HConst.OUTTAKE2_DIR);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        double lastTime = runtime.seconds();
        double lastPos = outtake1.getCurrentPosition();

        while (opModeIsActive()) {

            if(runtime.seconds() - lastTime > 0.1){
                vel = (outtake1.getCurrentPosition() - lastPos) / (28 * 0.1) * Math.PI * 2 * -1;
                lastTime = runtime.seconds();
                lastPos = outtake1.getCurrentPosition();
            }

            // Button A sets outtake1 to full power
            if (gamepad1.a) {
                outtake1.setPower(Math.signum(target) * 0.265 + (target - vel) * 0.015);
                outtake2.setPower(Math.signum(target) * 0.265 + (target - vel) * 0.015);
            } else {
                outtake1.setPower(0.0);
                outtake2.setPower(0.0);
            }

            if (gamepad1.dpad_up) target += 0.005;
            if (gamepad1.dpad_down) target -= 0.005;


            // Telemetry for verification
            telemetry.addData("Outtake1 Power", outtake1.getPower());
            telemetry.addData("Outtake2 Power", outtake2.getPower());
            telemetry.addData("Target Velocity in rad/s", target);
            telemetry.addData("Velocity in rad/s", vel);
            telemetry.update();
        }
    }
}
