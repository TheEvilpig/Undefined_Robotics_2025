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
import org.firstinspires.ftc.teamcode.util.HConst;

@TeleOp(name="Main_TeleOp", group="Linear OpMode")
public class mainTeleOp extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

    //DcMotorEx is basically like DcMotor but more advanced with more methods and capabilities
    private DcMotorEx frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive,
            intake, outtake1, outtake2, transfer;

    //Brakes
    private Servo rb, lb;

    //for brake system
    private boolean aPressed = false;
    private boolean servoPos = false;

    //limelight
    private Limelight3A limeLight;
    private LLResult latestResult;

    //needed to pair with limelight
    private IMU imu;

    //
    private boolean bPressed = false;
    private boolean outtakeOn = false;


    private boolean dpadUpPressed = false;
    private boolean allOn = false;
    private double rl = .8;
    long lastTime = System.nanoTime();

    //for reversing intake and transfer

    @Override

    public void runOpMode() {
        teleInit();

        waitForStart();

        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            updateDrivetrain();

            updateAuxiliaryMotors();

            latestResult = scanAprilTags();

            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.update();
        }

    }

    /**
     * Drives the robot chassis motors
     */
    private void updateDrivetrain(){
        double frontLeftPower;
        double backLeftPower;
        double frontRightPower;
        double backRightPower;

        if(gamepad1.left_bumper && getDistance(latestResult)!=-1) {
            double error = latestResult.getTx();
            double k = 0.13;
            double p = 0.006;

            double turnPower= p * error + Math.signum(error)*k;

            turnPower=Math.max(-.5,Math.min(.5,turnPower));

            if(Math.abs(error)<0.5)
                turnPower=0;
            frontLeftPower = turnPower;
            backLeftPower = turnPower;
            frontRightPower = -turnPower;
            backRightPower = -turnPower;

            telemetry.addData("Turn Power:",turnPower);
            telemetry.addData("Error:",error);

        }else{
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x * rl;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            frontLeftPower = (y + x + rx) / denominator;
            backLeftPower = (y - x + rx) / denominator;
            frontRightPower = (y - x - rx) / denominator;
            backRightPower = (y + x - rx) / denominator;

        }

        frontLeftDrive.setPower(frontLeftPower);
        backLeftDrive.setPower(backLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backRightDrive.setPower(backRightPower);
    }

    /**
     * Drives the intake, transfer, outtake motors and braking system servos.
     */
    private void updateAuxiliaryMotors(){
        //intake toggle
        /*
        if(gamepad1.x&&!xPressed){
            intakeOn=!intakeOn;
            xPressed=true;
        }
        else if(!gamepad1.x)
            xPressed=false;
        double intakePower = intakeOn ? 1 :0.0;
        double transferPower = intakeOn ? 1 : 0;

         */
        double intakePower = gamepad1.x ? 1 :0.0;
        double transferPower = gamepad1.x ? 1 : 0;
        //outtake toggle
        if(gamepad1.b&&!bPressed){
            outtakeOn=!outtakeOn;
            bPressed=true;
        }
        else if(!gamepad1.b)
            bPressed=false;
        double outtakePower = outtakeOn ?1:0;


        //conditions for 3 artifacts in robot

        if(gamepad1.dpad_up&&!dpadUpPressed){
            allOn=!allOn;
            dpadUpPressed=true;
        }
        else if(!gamepad1.dpad_up)
            dpadUpPressed=false;

        if(allOn){
            intakePower=.9;
            transferPower=.9;
            outtakePower= -.44;
        }

        if(gamepad1.dpad_down) {
            transferPower = -.44;
            outtakePower = -.44;
        }


        //using velocity in order to find the right power.
        /*
        double outtakeVelocity = gamepad1.b ? calculateOuttakeAngularVelocity():0;
        if(calculateOuttakeAngularVelocity()!=-1&&intakePower!=1){
            outtake1.setVelocity(outtakeVelocity,AngleUnit.RADIANS);
            outtake2.setVelocity(outtakeVelocity,AngleUnit.RADIANS);
            telemetry.addLine("Using setVelocity()");
        }
        else{
            outtake1.setPower(outtakePower);
            outtake2.setPower(outtakePower);
            telemetry.addLine("Using setPower()");
        }
         */
        //outtake1.setPower(outtakePower);
        //outtake2.setPower(outtakePower);

        outtake1.setVelocity(4 * outtakePower, AngleUnit.RADIANS);
        outtake2.setVelocity(4 * outtakePower, AngleUnit.RADIANS);

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
        telemetry.addData("Intake Power:", "%f", intake.getPower());
        telemetry.addData("Outtake Power:","%f", (outtake1.getPower()+outtake2.getPower())/2);
        telemetry.addData("Outtake1 Velocity:",outtake1.getVelocity(AngleUnit.RADIANS));
        telemetry.addData("Outtake2 Velocity:",outtake2.getVelocity(AngleUnit.RADIANS));

    }

    /**
     * intitialize, hardwaremap
     */
    private void teleInit(){
        //chassis
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, HConst.LEFT_FRONT);
        backLeftDrive = hardwareMap.get(DcMotorEx.class, HConst.LEFT_BACK);
        frontRightDrive = hardwareMap.get(DcMotorEx.class, HConst.RIGHT_FRONT);
        backRightDrive = hardwareMap.get(DcMotorEx.class, HConst.RIGHT_BACK);

        //shooting system
        intake = hardwareMap.get(DcMotorEx.class, HConst.INTAKE);

        //outtake 1 is in same orientation as previous outtake motor
        outtake1 = hardwareMap.get(DcMotorEx.class, HConst.OUTTAKE1);
        outtake2 = hardwareMap.get(DcMotorEx.class, HConst.OUTTAKE2);
        transfer = hardwareMap.get(DcMotorEx.class, HConst.TRANSFER);

        //braking system
        rb = hardwareMap.get(Servo.class, "rs");
        lb = hardwareMap.get(Servo.class, "ls");


        //April Tags
        limeLight = hardwareMap.get(Limelight3A.class,"Limelight");

        //imu
        imu = hardwareMap.get(IMU.class,"IMU");

        //directions
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setDirection(HConst.INTAKE_DIR);
        outtake1.setDirection(HConst.OUTTAKE1_DIR);
        outtake2.setDirection(HConst.OUTTAKE2_DIR);
        transfer.setDirection(HConst.TRANSFER_DIR);

        rb.setPosition(HConst.R_BRAKE_UP);
        lb.setPosition(HConst.L_BRAKE_UP);

        limeLight.setPollRateHz(100);
        limeLight.pipelineSwitch(1);
        limeLight.start();

        telemetry.addData("Status", "Initialized");
    }

    /**
     * Scans april tags on targets to help in robot allignment for accurate scoring trajectory.
     */
    private LLResult scanAprilTags(){
        YawPitchRollAngles ore = imu.getRobotYawPitchRollAngles();
        limeLight.updateRobotOrientation(ore.getYaw(AngleUnit.DEGREES));
        //latest limelight result
        LLResult result = limeLight.getLatestResult();
        if(result!=null && result.isValid()){
            Pose3D pose = result.getBotpose_MT2();
            telemetry.addData("Target X (Degrees)",result.getTx());
            telemetry.addData("Target Y (Degrees)",result.getTy());
            telemetry.addData("Botpose",pose.toString());
            return result;
        } else
            telemetry.addLine("No targets detected from Limelight.");

        return null;
    }

    /**
     * Calculates the distance the robot is from the target, using data from the limelight
     * @return the horizontal displacement/distance from the robot to the target.
     */
    private double getDistance(LLResult result){
        //height of target, 30 inches in meters
        double h2 = 0.762;
        //height of limelight in meters, 8 inches
        double h1 = 0.2032;
        //mounted angle of limelight in degrees
        double ang = 23;
        YawPitchRollAngles ore = imu.getRobotYawPitchRollAngles();
        limeLight.updateRobotOrientation(ore.getYaw(AngleUnit.DEGREES));
        //latest limelight result
        if(result!=null && result.isValid()){
            return (h2-h1)/(Math.tan(Math.toRadians(ang +result.getTy())));
        }
        return -1;
    }

    /**
     * Calculates the perfect angular velocity needed for the outtake system to undergo to launch artifacts into the target.
     * This does NOT account for air resistance/drag
     * @return velocity needed for robot to score into target.
     */
    private double calculateOuttakeAngularVelocity(LLResult result){
        YawPitchRollAngles ore = imu.getRobotYawPitchRollAngles();
        limeLight.updateRobotOrientation(ore.getYaw(AngleUnit.DEGREES));

        double dist = getDistance(result);

        if(result!=null && result.isValid()) {
            if(dist == -1)
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
            double num = 9.8*(Math.pow(getDistance(result)+xO,2));
            double denom = 2*Math.cos(Math.toRadians(fire))*((hT-h2)+(dist+xO)*Math.tan(Math.toRadians(fire)));
            double linearVel = Math.sqrt(num/denom);

            return linearVel/r;
        }
        return -1;
    }
}