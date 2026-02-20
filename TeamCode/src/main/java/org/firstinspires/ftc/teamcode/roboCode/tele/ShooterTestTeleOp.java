package org.firstinspires.ftc.teamcode.roboCode.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.util.HConst;
import org.firstinspires.ftc.teamcode.util.ToggleButton;

@TeleOp(name="Shooter_Only_Test", group="Test")
public class ShooterTestTeleOp extends LinearOpMode {

    private DcMotorEx outtake1;
    private DcMotorEx outtake2;

    private ToggleButton shooterToggle;

    // Raw power (no PID)
    private double power = 0.75;

    @Override
    public void runOpMode() {

        outtake1 = hardwareMap.get(DcMotorEx.class, HConst.OUTTAKE1);
        outtake2 = hardwareMap.get(DcMotorEx.class, HConst.OUTTAKE2);

        outtake1.setDirection(HConst.OUTTAKE1_DIR);
        outtake2.setDirection(HConst.OUTTAKE2_DIR);

        shooterToggle = new ToggleButton(() -> gamepad1.a);

        telemetry.addLine("Shooter Only TeleOp Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            shooterToggle.update();

            // Adjust power
            if (gamepad1.dpad_up) power += 0.05;
            if (gamepad1.dpad_down) power -= 0.05;

            power = Math.max(0, Math.min(1, power));

            if (shooterToggle.isToggled()) {
                outtake1.setPower(power);
                outtake2.setPower(power);
            } else {
                outtake1.setPower(0);
                outtake2.setPower(0);
            }

            telemetry.addLine("=== SHOOTER ONLY TEST ===");
            telemetry.addData("Shooter ON", shooterToggle.isToggled());
            telemetry.addData("Power", power);
            telemetry.update();
        }
    }
}
