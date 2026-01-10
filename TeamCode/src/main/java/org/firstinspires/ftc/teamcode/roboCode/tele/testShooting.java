/*package org.firstinspires.ftc.teamcode.roboCode.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.util.HConst;

@TeleOp(name="Test Shooting Manual PID Dual", group="Linear OpMode")
public class testShooting extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotorEx intake, outtake1, outtake2, transfer;

    private boolean bPressed = false;
    private boolean outtakeOn = false;
    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;

    private double targetVelocity = 150;  // rad/s
    private final double VELOCITY_INCREMENT = 10;

    // Manual PID constants
    private final double kP = 0.015;
    private final double kF = 0.002;
    private final double kK = 0.2;
    private final double kI = 0.002;

    // Encoder tracking
    private int lastPosition = 0;
    private double lastTime = 0.0;
    private double lastVelUpdate = 0.0;
    private double accumulatedDifference = 0;

    // Smoothed velocity
    private double measuredVelocity = 0;

    // Update interval for velocity (in seconds)
    private final double VELOCITY_UPDATE_INTERVAL = 0.1;

    @Override
    public void runOpMode() {

        teleInit();

        waitForStart();
        runtime.reset();

        lastPosition = outtake1.getCurrentPosition();
        lastTime = runtime.seconds();
        lastVelUpdate = runtime.seconds();

        while (opModeIsActive()) {

            updateAuxiliaryMotors();

            telemetry.addData("Status", "Run Time: " + runtime.seconds());
            telemetry.update();
        }
    }

    private void updateAuxiliaryMotors() {

        double intakePower = gamepad1.x ? 1 : 0.0;
        double transferPower = gamepad1.x ? 1 : 0.0;

        // Outtake toggle
        if (gamepad1.b && !bPressed) {
            outtakeOn = !outtakeOn;
            bPressed = true;
        } else if (!gamepad1.b) {
            bPressed = false;
        }

        // Adjust target velocity
        if (gamepad1.dpad_up && !dpadUpPressed) {
            targetVelocity += VELOCITY_INCREMENT;
            dpadUpPressed = true;
        } else if (!gamepad1.dpad_up) dpadUpPressed = false;

        if (gamepad1.dpad_down && !dpadDownPressed) {
            targetVelocity -= VELOCITY_INCREMENT;
            if (targetVelocity < 0) targetVelocity = 0;
            dpadDownPressed = true;
        } else if (!gamepad1.dpad_down) dpadDownPressed = false;

        // Update velocity only every VELOCITY_UPDATE_INTERVAL
        double currentTime = runtime.seconds();
        if (currentTime - lastVelUpdate >= VELOCITY_UPDATE_INTERVAL) {
            int currentPosition = outtake1.getCurrentPosition();
            double deltaTime = currentTime - lastTime;
            if (deltaTime <= 0) deltaTime = 0.001; // prevent divide by zero

            double ticksPerRev = 28;
            measuredVelocity = (currentPosition - lastPosition) / (ticksPerRev * deltaTime) * 2.0 * Math.PI * -1;

            accumulatedDifference += targetVelocity - measuredVelocity;
            lastPosition = currentPosition;
            lastTime = currentTime;
            lastVelUpdate = currentTime;
        }

        // Apply manual PID / feedforward if outtake is on
        if (outtakeOn) {
            double power = kF * targetVelocity + kP * (targetVelocity - measuredVelocity) + kK + kI * accumulatedDifference;
            power = Math.max(-1.0, Math.min(1.0, power));

            outtake1.setPower(power);
            outtake2.setPower(power);
        } else {
            outtake1.setPower(0.0);
            outtake2.setPower(0.0);
        }

        transfer.setPower(transferPower);
        intake.setPower(intakePower);

        // Telemetry
        telemetry.addLine("===Testing intake and outtake===")
                .addData("Target Velocity (rad/s)", "%.1f", targetVelocity)
                .addData("Outtake State", outtakeOn ? "ON" : "OFF")
                .addData("Master Motor Power", outtake1.getPower())
                .addData("Follower Motor Power", outtake2.getPower())
                .addData("Measured Velocity (rad/s)", measuredVelocity);
    }

    private void teleInit() {
        intake = hardwareMap.get(DcMotorEx.class, HConst.INTAKE);
        outtake1 = hardwareMap.get(DcMotorEx.class, HConst.OUTTAKE1);
        outtake2 = hardwareMap.get(DcMotorEx.class, HConst.OUTTAKE2);
        transfer = hardwareMap.get(DcMotorEx.class, HConst.TRANSFER);
        intake.setDirection(HConst.INTAKE_DIR);
        outtake1.setDirection(HConst.OUTTAKE1_DIR);
        outtake2.setDirection(HConst.OUTTAKE2_DIR);
        transfer.setDirection(HConst.TRANSFER_DIR);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
}*/

package org.firstinspires.ftc.teamcode.roboCode.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.DcMotorSystem;
import org.firstinspires.ftc.teamcode.util.HConst;

@TeleOp(name = "Test Shooting Manual PID Dual", group = "Linear OpMode")
public class testShooting extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotorEx intake, transfer;
    private DcMotorEx outtake1, outtake2;

    private DcMotorSystem shooter;

    private boolean bPressed = false;
    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;


    private double targetVelocity = 150; // rad/s
    private static final double VELOCITY_INCREMENT = 10;

    @Override
    public void runOpMode() {

        teleInit();

        shooter = new DcMotorSystem(
                outtake2,
                28,     // ticks per rev
                0.01, // velocity update interval
                telemetry
        );
        shooter.addFollower(outtake1);

        shooter.setPID(
                0.0015,  // kP
                0.0005,  // kI
                0.002,  // kF
                0.1     // kStatic
        );

        shooter.setTargetVelocity(targetVelocity);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            updateAuxiliaryMotors();
            shooter.update();

            telemetry.addData("Status", "Run Time: %.1f", runtime.seconds());
            telemetry.addData("Target Velocity (rad/s)", shooter.getTargetVelocity());
            telemetry.addData("Measured Velocity (rad/s)", shooter.getMeasuredVelocity());
            telemetry.addData("Shooter Enabled", shooter.isEnabled());
            telemetry.addData("pos1", outtake1.getCurrentPosition());
            telemetry.addData("pos2", outtake2.getCurrentPosition());
            telemetry.addData("power", shooter.power);
            telemetry.update();
        }
    }

    private void updateAuxiliaryMotors() {

        // Intake / transfer (unchanged)
        double intakePower = gamepad1.x ? 1.0 : 0.0;
        double transferPower = gamepad1.x ? 1.0 : 0.0;

        intake.setPower(intakePower);
        transfer.setPower(transferPower);

        // Shooter toggle (same as before)
        if (gamepad1.b && !bPressed) {

            if (shooter.isEnabled()) shooter.disable();
            else shooter.enable();
            bPressed = true;
        } else if (!gamepad1.b) {
            bPressed = false;
        }

        // Velocity tuning (same increments)
        if (gamepad1.dpad_up && !dpadUpPressed) {

            targetVelocity += VELOCITY_INCREMENT;
            shooter.setTargetVelocity(targetVelocity);
            dpadUpPressed = true;
        } else if (!gamepad1.dpad_up) {
            dpadUpPressed = false;
        }

        if (gamepad1.dpad_down && !dpadDownPressed) {
            targetVelocity -= VELOCITY_INCREMENT;
            if (targetVelocity < 0) targetVelocity = 0;
            shooter.setTargetVelocity(targetVelocity);
            dpadDownPressed = true;
        } else if (!gamepad1.dpad_down) {
            dpadDownPressed = false;
        }
    }

    private void teleInit() {

        intake = hardwareMap.get(DcMotorEx.class, HConst.INTAKE);
        outtake1 = hardwareMap.get(DcMotorEx.class, HConst.OUTTAKE1);
        outtake2 = hardwareMap.get(DcMotorEx.class, HConst.OUTTAKE2);
        transfer = hardwareMap.get(DcMotorEx.class, HConst.TRANSFER);

        intake.setDirection(HConst.INTAKE_DIR);
        outtake1.setDirection(HConst.OUTTAKE1_DIR);
        outtake2.setDirection(HConst.OUTTAKE2_DIR);

        transfer.setDirection(HConst.TRANSFER_DIR);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
}
