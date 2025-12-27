package org.firstinspires.ftc.teamcode.roboCode.test_demo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.HConst;

@TeleOp(name="Manual PID Shooter Test", group="Linear OpMode")
public class ManualPIDShooterTest extends LinearOpMode {

    // Motors
    private DcMotorEx outtake1, outtake2;

    // Timer
    private ElapsedTime runtime = new ElapsedTime();

    // Encoder constants
    private final double TPR = 28.0; // Bare GoBilda 5000-0002-4008
    private double lastPosition = 0;
    private double lastTime = 0;

    // Velocity update interval
    private final double VELOCITY_UPDATE_MS = 20; // 20ms
    private double lastVelUpdate = 0;

    // PID constants (example values)
    private final double kP = 0.5;
    private final double kI = 0.0;
    private final double kD = 0.0;
    private final double kF = 0.1;

    private double integral = 0;
    private double lastError = 0;

    // Target velocity (rad/s)
    private double targetVelocity = 0.5;

    @Override
    public void runOpMode() {

        // Initialize motors
        outtake1 = hardwareMap.get(DcMotorEx.class, HConst.OUTTAKE1);
        outtake2 = hardwareMap.get(DcMotorEx.class, HConst.OUTTAKE2);

        outtake1.setDirection(HConst.OUTTAKE1_DIR);
        outtake2.setDirection(HConst.OUTTAKE2_DIR);

        outtake1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        lastTime = runtime.seconds();
        lastPosition = outtake1.getCurrentPosition();

        while (opModeIsActive()) {

            double currentTime = runtime.seconds();
            double currentPosition = outtake1.getCurrentPosition();

            // Update velocity every VELOCITY_UPDATE_MS
            if ((currentTime - lastVelUpdate) * 1000 >= VELOCITY_UPDATE_MS) {

                double deltaTime = currentTime - lastTime;
                double deltaTicks = currentPosition - lastPosition;

                double velocityTicksPerSec = deltaTicks / deltaTime;
                double velocityRadPerSec = (velocityTicksPerSec / TPR) * 2 * Math.PI;

                // Simple manual PID
                double error = targetVelocity - velocityRadPerSec;
                integral += error * deltaTime;
                double derivative = (error - lastError) / deltaTime;
                double power = targetVelocity * kF;//kP * error + kI * integral + kD * derivative;

                // Clamp power 0–1
                power = Math.max(0, Math.min(1, power));

                // Apply to both motors
                outtake1.setPower(power);
                outtake2.setPower(power);

                // Update telemetry
                telemetry.addLine("===Manual PID Shooter Test===");
                telemetry.addData("Target Velocity (rad/s)", "%.2f", targetVelocity);
                telemetry.addData("Measured Velocity (rad/s)", "%.2f", velocityRadPerSec);
                telemetry.addData("Motor Power", "%.2f", power);
                telemetry.addData("Motor Power2", "%.2f", outtake1.getPower());

                telemetry.update();

                // Update tracking variables
                lastPosition = currentPosition;
                lastTime = currentTime;
                lastError = error;
                lastVelUpdate = currentTime;
            }

            // Optional: adjust target with gamepad
            if (gamepad1.dpad_up) targetVelocity += 0.001;
            if (gamepad1.dpad_down) targetVelocity -= 0.001;
            if (targetVelocity < 0) targetVelocity = 0;
        }
    }
}
