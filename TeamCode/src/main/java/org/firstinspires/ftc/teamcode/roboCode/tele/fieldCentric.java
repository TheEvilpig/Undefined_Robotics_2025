package org.firstinspires.ftc.teamcode.roboCode.tele;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.HConst;

@TeleOp(name = "Field Centric Tester", group = "Test")
public class fieldCentric extends LinearOpMode {

    private DcMotorEx frontLeft, backLeft, frontRight, backRight;
    private IMU imu;

    private double fieldHeadingOffset = 0;

    @Override
    public void runOpMode() {

        // Drivetrain
        frontLeft  = hardwareMap.get(DcMotorEx.class, HConst.LEFT_FRONT);
        backLeft   = hardwareMap.get(DcMotorEx.class, HConst.LEFT_BACK);
        frontRight = hardwareMap.get(DcMotorEx.class, HConst.RIGHT_FRONT);
        backRight  = hardwareMap.get(DcMotorEx.class, HConst.RIGHT_BACK);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // IMU
        imu = hardwareMap.get(IMU.class, "IMU");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );

        imu.initialize(parameters);

        telemetry.addLine("Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Reset yaw completely
            if (gamepad1.options) {
                imu.resetYaw();
            }

            // Set current direction as "field forward"
            if (gamepad1.a) {
                setFieldCentricHeading(0);
            }

            // Joystick inputs
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // Get heading
            double rawHeading = imu.getRobotYawPitchRollAngles()
                    .getYaw(AngleUnit.RADIANS);

            double botHeading = normalizeAngle(rawHeading - fieldHeadingOffset);

            // Rotate joystick vector
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX *= 1.1; // strafe correction

            double denominator = Math.max(
                    Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

            double fl = (rotY + rotX + rx) / denominator;
            double bl = (rotY - rotX + rx) / denominator;
            double fr = (rotY - rotX - rx) / denominator;
            double br = (rotY + rotX - rx) / denominator;

            frontLeft.setPower(fl);
            backLeft.setPower(bl);
            frontRight.setPower(fr);
            backRight.setPower(br);

            telemetry.addData("Raw Heading (deg)",
                    imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("Offset (deg)",
                    Math.toDegrees(fieldHeadingOffset));
            telemetry.addData("Bot Heading (deg)",
                    Math.toDegrees(botHeading));
            telemetry.update();
        }
    }

    private void setFieldCentricHeading(double desiredHeadingDegrees) {
        double currentHeading = imu.getRobotYawPitchRollAngles()
                .getYaw(AngleUnit.RADIANS);
        double desiredHeading = Math.toRadians(desiredHeadingDegrees);

        fieldHeadingOffset = currentHeading - desiredHeading;
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
