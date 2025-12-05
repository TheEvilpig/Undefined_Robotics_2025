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

@TeleOp(name="Main_TeleOp", group="Linear OpMode")
public class mainTeleOp extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime;
    //DcMotorEx is basically like DcMotor but more advanced with more methods and capabilities
    private DcMotorEx frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive,
            intake, outtake1, outtake2, transfer;
    //Brakes
    private Servo b1, b2;
    //for brake system
    private boolean aPressed = false;
    private boolean servoPos = false;


    private Limelight3A limeLight;
    //needed to pair with limelight
    private IMU imu;
    private boolean rBumper = false;
    private boolean lBumper = false;
    private double rl = .8;

    //braking servo positions
    double B2U = 0.19;
    double B2C = 0;

    double B1U = 0.83;
    double B1C = 1;
    //for reversing intake and transfer
    private double flip =1;

    @Override

    public void runOpMode() {
        ElapsedTime runtime = new ElapsedTime();
        //chassis
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "lf");
        backLeftDrive = hardwareMap.get(DcMotorEx.class, "lb");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "rf");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "rb");
        //shooting system
        intake = hardwareMap.get(DcMotorEx.class,"Intake");
        //outtake 1 is in same orientation as previous outtake motor
        outtake1 = hardwareMap.get(DcMotorEx.class,"Outtake1");
        outtake2 = hardwareMap.get(DcMotorEx.class,"Outtake2");
        transfer = hardwareMap.get(DcMotorEx.class,"Transfer");
        //braking system
        b2 = hardwareMap.get(Servo.class, "ls");
        b1 = hardwareMap.get(Servo.class, "rs");
        //April Tags
        limeLight = hardwareMap.get(Limelight3A.class,"Limelight");
        //imu
        imu = hardwareMap.get(IMU.class,"IMU");

        //directions
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.REVERSE);
        outtake1.setDirection(DcMotor.Direction.REVERSE);
        outtake2.setDirection(DcMotor.Direction.FORWARD);
        transfer.setDirection(DcMotor.Direction.REVERSE);

        b1.setPosition(B1U);
        b2.setPosition(B2U);
        limeLight.setPollRateHz(100);
        limeLight.pipelineSwitch(1);
        limeLight.start();


        telemetry.addData("Status", "Initialized");

        //</editor-fold>

        waitForStart();

        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            updateDrivetrain();

            updateAuxiliaryMotors();

            scanAprilTags();
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addLine("===Testing intake and outtake===");
            telemetry.addData("Intake Power:", "%f", intake.getPower());
            telemetry.addData("Outtake Power:","%f", (outtake1.getPower()+outtake2.getPower())/2);

            scanAprilTags();

            telemetry.update();
        }

    }

    /**
     * Drives the robot chassis motors
     */
    private void updateDrivetrain(){
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x * rl;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;
/*
        Isolated to test chassis motors
        frontLeftPower = gamepad1.dpad_up ? 1.0:0;
        frontRightPower = gamepad1.dpad_right ? 1.0:0;
        backRightPower = gamepad1.dpad_down ? 1.0:0;
        backLeftPower = gamepad1.dpad_left ? 1.0:0;
*/


        frontLeftDrive.setPower(frontLeftPower);
        backLeftDrive.setPower(backLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backRightDrive.setPower(backRightPower);

    }

    /**
     * Drives the intake, transfer, outtake motors and braking system servos.
     */
    private void updateAuxiliaryMotors(){
        if (gamepad1.right_bumper&&!rBumper)
            flip = -flip;  // Flip once per press
        rBumper = gamepad1.right_bumper;

        double intakePower = gamepad1.x ? flip :0.0;
        double transferPower = gamepad1.y ? flip :0;
        //using velocity in order to find the right power.
        double outtakeVelocity = gamepad1.b ? calculateOuttakeAngularVelocity():0;
        double outtakePower = gamepad1.b ? 1:0;
        if(calculateOuttakeAngularVelocity()!=-1){
            outtake1.setVelocity(outtakeVelocity,AngleUnit.RADIANS);
            outtake2.setVelocity(outtakeVelocity,AngleUnit.RADIANS);
        }
        else{
            outtake1.setPower(outtakePower);
            outtake2.setPower(outtakePower);
        }

        if (gamepad1.a && !aPressed) {
            servoPos = !servoPos;
            aPressed = true;
        }

        if (!gamepad1.a) aPressed = false;

        if(servoPos){
            b1.setPosition(B1C);
            b2.setPosition(B2C);
        }else {
            b1.setPosition(B1U);
            b2.setPosition(B2U);
        }



        intake.setPower(intakePower);

        transfer.setPower(transferPower);

    }

    /**
     * Scans april tags on targets to help in robot allignment for accurate scoring trajectory.
     */
    private void scanAprilTags(){
        YawPitchRollAngles ore = imu.getRobotYawPitchRollAngles();
        limeLight.updateRobotOrientation(ore.getYaw(AngleUnit.DEGREES));
        //latest limelight result
        LLResult result = limeLight.getLatestResult();
        if(result!=null && result.isValid()){
            Pose3D pose = result.getBotpose_MT2();
            telemetry.addData("Target X (Degrees)",result.getTx());
            telemetry.addData("Target Y (Degrees)",result.getTy());
            telemetry.addData("Botpose",pose.toString());

        }
        else
            telemetry.addLine("No targets detected from Limelight.");

        if(gamepad1.left_bumper && getDistance()!=-1) {
            //robot too much to the right of the target
            double power = .5;
            if (result.getTx() > getMaxDisplacementAngle()) {
                frontLeftDrive.setPower(-power);
                backLeftDrive.setPower(-power);
                frontRightDrive.setPower(power);
                backRightDrive.setPower(power);
            }
            //robot too much to the left of the target
            else if (result.getTx() < -getMaxDisplacementAngle()) {
                frontLeftDrive.setPower(-power);
                backLeftDrive.setPower(-power);
                frontRightDrive.setPower(power);
                backRightDrive.setPower(power);
            } else {
                frontLeftDrive.setPower(0);
                backLeftDrive.setPower(0);
                frontRightDrive.setPower(0);
                backRightDrive.setPower(0);
            }
            telemetry.addLine("Aligning...");
        }

    }
    //xL

    /**
     * Calculates the distance the robot is from the target, using data from the limelight
     * @return the horizontal displacement/distance from the robot to the target.
     */
    private double getDistance(){
        //height of target, 30 inches in meters
        double h2 = 0.762;
        //height of limelight in meters
        double h1 = 0;
        //mounted angle of limelight in degrees
        double ang = 23;
        YawPitchRollAngles ore = imu.getRobotYawPitchRollAngles();
        limeLight.updateRobotOrientation(ore.getYaw(AngleUnit.DEGREES));
        //latest limelight result
        LLResult result = limeLight.getLatestResult();
        if(result!=null && result.isValid()){
            return (h2-h1)/(Math.tan(Math.toRadians(ang +result.getTy())));
        }
        return -1;
    }

    /**
     * Calculates the maximum allowed displacement angle the robot could have from the target it is aiming at.
     * Robot will rotate left if it is over the angle and align right if it is under the angle
     * @return max displacement angle robot needs in order to fire accurately.
     */
    private double getMaxDisplacementAngle(){
        YawPitchRollAngles ore = imu.getRobotYawPitchRollAngles();
        limeLight.updateRobotOrientation(ore.getYaw(AngleUnit.DEGREES));
        //latest limelight result
        LLResult result = limeLight.getLatestResult();
        if(result!=null && result.isValid()) {
            if(getDistance()==-1)
                return -1;
            return Math.atan(16 / getDistance());
        }
        return -1;
    }

    /**
     * Calculates the perfect angular velocity needed for the outtake system to undergo to launch artifacts into the target.
     * This does NOT account for air resistance/drag
     * @return velocity needed for robot to score into target.
     */
    private double calculateOuttakeAngularVelocity(){
        YawPitchRollAngles ore = imu.getRobotYawPitchRollAngles();
        limeLight.updateRobotOrientation(ore.getYaw(AngleUnit.DEGREES));
        //latest limelight result
        LLResult result = limeLight.getLatestResult();
        if(result!=null && result.isValid()) {
            if(getDistance()==-1)
                return -1;
            //radius of outtake wheel
            double r = 1;
            //height of target, 40 inches in meters
            double h2 = 1.016;
            //height of outtake launch
            double hT = 0;
            //displacement of limelight from outtake, assuming they are both in the middle
            double xO = 0;
            //angle of artifact firing from outtake, in degrees.
            double fire = 0;
            //numerator of final equation
            double num = 9.8*(Math.pow(getDistance()+xO,2));
            double denom = 2*Math.cos(Math.toRadians(fire))*((hT-h2)+(getDistance()+xO)*Math.tan(Math.toRadians(fire)));
            double linearVel = Math.sqrt(num/denom);

            return linearVel/r;
        }
        return -1;
    }

}