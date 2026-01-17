package org.firstinspires.ftc.teamcode.roboCode.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.util.HConst;
import org.firstinspires.ftc.teamcode.util.ToggleButton;

@TeleOp(name = "Hold Servo Test", group = "Test")
public class HoldServoTeleOp extends LinearOpMode {

    private Servo hold;

    // Toggle on button A
    private final ToggleButton holdToggle =
            new ToggleButton(() -> gamepad1.a);

    @Override
    public void runOpMode() {

        hold = hardwareMap.get(Servo.class, HConst.HOLD);

        telemetry.addLine("Hold Servo TeleOp Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            holdToggle.update();

            boolean holding = holdToggle.isToggled();

            double holdPos = holding
                    ? HConst.HOLD_ACTIVE
                    : HConst.HOLD_INACTIVE;

            hold.setPosition(holdPos);

            telemetry.addData("Hold State", holding ? "ACTIVE" : "INACTIVE");
            telemetry.addData("Servo Position", holdPos);
            telemetry.update();
        }
    }
}