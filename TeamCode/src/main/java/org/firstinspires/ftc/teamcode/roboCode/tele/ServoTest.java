package org.firstinspires.ftc.teamcode.roboCode.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="SERVO TEST", group="Linear OpMode")
public class ServoTest extends LinearOpMode{
    private Servo ser;
    @Override
    public void runOpMode() {
        ser = hardwareMap.get(Servo.class,"ser");
        ser.setDirection(Servo.Direction.FORWARD);
        waitForStart();
        while (opModeIsActive()) {
            ser.setPosition(1);
            sleep(500);
            ser.setPosition(0);
            sleep(500);
            ser.setPosition(.5);
            sleep(5000);
        }
    }
}
