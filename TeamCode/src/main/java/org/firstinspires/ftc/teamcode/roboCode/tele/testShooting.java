package org.firstinspires.ftc.teamcode.roboCode.tele;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.HConst;

@TeleOp(name="Test Shooting", group="Linear OpMode")
public class testShooting extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

    //DcMotorEx is basically like DcMotor but more advanced with more methods and capabilities
    private DcMotorEx intake, outtake1, outtake2, transfer;

    //Brakes
    private Servo rb, lb;

    //for brake system
    private boolean aPressed = false;
    private boolean servoPos = false;


    //
    private boolean bPressed = false;
    private boolean outtakeOn = false;


    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;

    long lastTime = System.nanoTime();

    // Configuration variables
    private double targetVelocity = 4.0;  // Default velocity in rad/s
    private final double VELOCITY_INCREMENT = 0.5;  // How much to change per button press

    @Override

    public void runOpMode() {
        teleInit();

        waitForStart();

        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            updateAuxiliaryMotors();

            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.update();
        }

    }

    /**
     * Drives the intake, transfer, outtake motors and braking system servos.
     */
    private void updateAuxiliaryMotors(){
        double intakePower = gamepad1.x ? 1 :0.0;
        double transferPower = gamepad1.x ? 1 : 0;

        //outtake toggle
        if (gamepad1.b && !bPressed) {
            outtakeOn = !outtakeOn;
            bPressed = true;
        } else if (!gamepad1.b)
            bPressed = false;

        // Adjust target velocity with dpad up/down
        if (gamepad1.dpad_up && !dpadUpPressed) {
            targetVelocity += VELOCITY_INCREMENT;
            dpadUpPressed = true;
        } else if (!gamepad1.dpad_up) {
            dpadUpPressed = false;
        }

        if (gamepad1.dpad_down && !dpadDownPressed) {
            targetVelocity -= VELOCITY_INCREMENT;
            if (targetVelocity < 0) targetVelocity = 0;  // Don't go negative
            dpadDownPressed = true;
        } else if (!gamepad1.dpad_down) {
            dpadDownPressed = false;
        }

        // Apply target velocity when outtake is on
        if (outtakeOn) {
            outtake1.setVelocity(targetVelocity, AngleUnit.RADIANS);
            outtake2.setVelocity(targetVelocity, AngleUnit.RADIANS);
        } else {
            outtake1.setVelocity(0, AngleUnit.RADIANS);
            outtake2.setVelocity(0, AngleUnit.RADIANS);
        }

        // Brake system toggle
        if (gamepad1.a && !aPressed) {
            servoPos = !servoPos;
            aPressed = true;
        }

        if (!gamepad1.a) aPressed = false;

        if(servoPos){
            rb.setPosition(HConst.R_BRAKE_DOWN);
            lb.setPosition(HConst.L_BRAKE_DOWN);
        }else {
            rb.setPosition(HConst.R_BRAKE_UP);
            lb.setPosition(HConst.L_BRAKE_UP);
        }

        transfer.setPower(transferPower);
        intake.setPower(intakePower);

        telemetry.addLine("===Testing intake and outtake===");
        telemetry.addData("Target Velocity (rad/s)", "%.1f", targetVelocity);
        telemetry.addData("Outtake State", outtakeOn ? "ON" : "OFF");
        telemetry.addData("Intake Power:", "%f", intake.getPower());
        telemetry.addData("Outtake Power:","%f", (outtake1.getPower()+outtake2.getPower())/2);
        telemetry.addData("Outtake1 Velocity:",outtake1.getVelocity(AngleUnit.RADIANS));
        telemetry.addData("Outtake2 Velocity:",outtake2.getVelocity(AngleUnit.RADIANS));

    }


    /**
     * intitialize, hardwaremap
     */
    private void teleInit(){

        //shooting system
        intake = hardwareMap.get(DcMotorEx.class, HConst.INTAKE);

        //outtake 1 is in same orientation as previous outtake motor
        outtake1 = hardwareMap.get(DcMotorEx.class, HConst.OUTTAKE1);
        outtake2 = hardwareMap.get(DcMotorEx.class, HConst.OUTTAKE2);
        transfer = hardwareMap.get(DcMotorEx.class, HConst.TRANSFER);



        intake.setDirection(HConst.INTAKE_DIR);
        outtake1.setDirection(HConst.OUTTAKE1_DIR);
        outtake2.setDirection(HConst.OUTTAKE2_DIR);
        transfer.setDirection(HConst.TRANSFER_DIR);

        rb.setPosition(HConst.R_BRAKE_UP);
        lb.setPosition(HConst.L_BRAKE_UP);


        telemetry.addData("Status", "Initialized");
    }

}