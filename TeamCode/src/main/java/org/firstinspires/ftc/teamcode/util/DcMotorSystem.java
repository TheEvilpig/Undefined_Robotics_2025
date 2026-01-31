package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

public class DcMotorSystem {

    private final DcMotorEx master;
    private final List<DcMotorEx> followers = new ArrayList<>();

    // PID + feedforward
    private double kP, kI, kF, kS;

    public int num = 0;

    private static final double A = 84.3661059236;
    private static final double B = 0.0794062969795;
    private static final double C = 293.987179162;

    // State
    private double targetVelocity = 0.0; // rad/s
    private double measuredVelocity = 0.0;
    private double integralSum = 0.0;

    // Encoder tracking
    private int lastPosition = 0;
    private double lastTime = 0.0;
    public double power = 0;

    // Config
    private final double ticksPerRev;
    private final double velocityUpdateInterval;
    private double lastVelUpdate = 0.0;

    private Telemetry tele;

    private boolean enabled = false;

    private final ElapsedTime timer = new ElapsedTime();

    public DcMotorSystem(
            DcMotorEx master,
            double ticksPerRev,
            double velocityUpdateInterval,
            Telemetry t
    ) {
        this.master = master;
        this.ticksPerRev = ticksPerRev;
        this.velocityUpdateInterval = velocityUpdateInterval;
        tele = t;

        lastPosition = master.getCurrentPosition();
        lastTime = timer.seconds();
    }

    /* ---------------- CONFIG ---------------- */

    public void addFollower(DcMotorEx motor) {
        followers.add(motor);
    }

    public void setPID(double kP, double kI, double kF, double kS) {
        this.kP = kP;
        this.kI = kI;
        this.kF = kF;
        this.kS = kS;
    }

    public void setTargetVelocity(double radPerSec) {
        targetVelocity = Math.max(0, radPerSec);
    }

    public void enable() {
        enabled = true;
        integralSum = 0.0;
    }

    public void disable() {
        enabled = false;
        setAllPower(0.0);
    }

    /* ---------------- UPDATE LOOP ---------------- */

    public void update() {
        double currentTime = timer.seconds();

        // Update velocity at fixed interval
        if (currentTime - lastVelUpdate >= velocityUpdateInterval) {
            updateVelocity(currentTime);
            lastVelUpdate = currentTime;
        }

        if (!enabled) return;

        double error = targetVelocity - measuredVelocity;
        integralSum += error * velocityUpdateInterval;

        double output =
                kF * targetVelocity +
                        kP * error +
                        kI * integralSum +
                        (targetVelocity != 0 ? kS : 0);

        output = clamp(output, -1.0, 1.0);
        setAllPower(output);
    }

    private void updateVelocity(double currentTime) {
        int currentPosition = master.getCurrentPosition();
        double deltaTime = currentTime - lastTime;
        if (deltaTime <= 0) deltaTime = 0.001;

        measuredVelocity =
                (currentPosition - lastPosition)
                       / (ticksPerRev * deltaTime)
                        * 2.0 * Math.PI;



        lastPosition = currentPosition;
        lastTime = currentTime;

        num++;
    }

    private void setAllPower(double power) {
        master.setPower(power);
        for (DcMotorEx follower : followers) {
            follower.setPower(power);
        }

        this.power = power;
    }

    /* ---------------- UTIL - (may relocate to a different class later) ---------------- */


    //change this to have params for everything so you can change tvelocities in main with click fo a
    //button to do efficient testing
    public double computeTargetVelocityFromDistance(double x) {
        return 0.00817963 * Math.pow(x, 2) -0.570922 * x + 254.31885;
    }


    /* ---------------- ACCESSORS ---------------- */

    public double getMeasuredVelocity() {
        return measuredVelocity;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public boolean isEnabled() {
        return enabled;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
