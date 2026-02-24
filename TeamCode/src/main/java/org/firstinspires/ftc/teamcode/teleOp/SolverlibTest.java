package org.firstinspires.ftc.teamcode.teleOp;


public class SolverlibTest {
/*

    // Shooter motors
    private DcMotorEx leftOuttake;
    private DcMotorEx rightOuttake;

    // PIDF controller
    private PIDFController pidf;

    // Target velocity (ticks/sec or RPM — just be consistent)
    private double targetVelocity = 0;

    // Cached gains (nice for telemetry & tuning)
    public static double kP, kI, kD, kF;

    // =========================
    // Constructor
    // =========================
    public SolverlibTest(
            DcMotorEx leftOuttake,
            DcMotorEx rightOuttake,
            double kP,
            double kI,
            double kD,
            double kF
    ) {
        this.leftOuttake = leftOuttake;
        this.rightOuttake = rightOuttake;

        // Store gains
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;

        // Create PIDF controller
        pidf = new PIDFController(kP, kI, kD, kF);

        // Motor configuration
        leftOuttake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightOuttake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        leftOuttake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightOuttake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        // Reverse one motor if needed
        leftOuttake.setDirection(DcMotorEx.Direction.REVERSE);
    }

    // =========================
    // Set target shooter velocity
    // =========================
    public void setTargetVelocity(double velocity) {
        targetVelocity = velocity;
    }

    // =========================
    // Main control loop
    // Call this every OpMode loop
    // =========================
    public void update() {
        double currentVelocity = getAverageVelocity();

        double output = pidf.calculate(
                currentVelocity,
                targetVelocity
        );

        leftOuttake.setPower(output);
        rightOuttake.setPower(output);
    }

    // =========================
    // Stop shooter
    // =========================
    public void stop() {
        targetVelocity = 0;
        leftOuttake.setPower(0);
        rightOuttake.setPower(0);
        pidf.reset();
    }

    // =========================
    // PIDF tuning (live)
    // =========================
    public void setPIDF(double p, double i, double d, double f) {
        pidf.setPIDF(p, i, d, f);

        kP = p;
        kI = i;
        kD = d;
        kF = f;
    }

    // =========================
    // Velocity helpers
    // =========================
    private double getAverageVelocity() {
        return (leftOuttake.getVelocity() + rightOuttake.getVelocity()) / 2.0;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getCurrentVelocity() {
        return getAverageVelocity();
    }

    public double getError() {
        return pidf.getPositionError(); // velocity error
    }

 */
}