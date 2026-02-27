package org.firstinspires.ftc.teamcode.teleOp.LCTeleOps;


import static com.pedropathing.paths.HeadingInterpolator.linearFromPoint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.EndPose;

import java.util.concurrent.TimeUnit;
import java.util.function.Supplier;

@Configurable
@TeleOp
public class CloseShooterLC extends OpMode {
    Deadline gamepadRateLimit = new Deadline(250, TimeUnit.MILLISECONDS);
    private Follower follower;
    //Sloth
    DcMotor intake;
    DcMotor transfer;
    DcMotorEx leftOuttake, rightOuttake;
    DcMotor leftFront, leftBack, rightFront, rightBack;
    public Servo servo;
    public Servo servo2;
    //Initialize Variables
    /*
    (Button) Initialize Period, before you press start on your program.
     */
    ElapsedTime transferTime = new ElapsedTime();
    public static double ticksPerSecond = 1200;
    //1500 is far
    //1250 is close
    public static double servoPos = 0.125;
    //0.335 is far
    //0.393 is close
    public static double minimum = 0;
    //0 is close
    //1480 is far
    // static double transferPower = 0.75;
    //1 is close
    //0.85 is far
    double maxTransfer = 1;
    double minTransfer = 0.82;
    double maxHood = 0.7;
    double minHood = 0.55;
    private Supplier<PathChain> pathChain;

    static final double targetX = 144;
    static final double targetY = 144;
    double minVelocity = 1005;
    double maxVelocity = 1615;
    //1550
    private boolean automatedDrive;

    double minDistance = 30.941125497;
    double maxDistance = 148;
    public static double transferPower = 1;
    //1 is close
    //0.85 is far
    boolean lastA = false;

    public enum shooterStates {
        CLOSE,
        FAR
    }
    private shooterStates mode = shooterStates.CLOSE;
    //233,0,0.0525,14.8
    public static PIDFCoefficients coeffs = new PIDFCoefficients(275, 0, 0.05, 15.2);
//450, 0, 0.012, 11.7
@Override
public void init() {
    follower = Constants.createFollower(hardwareMap);
    //follower.setStartingPose(endPose == null ? new Pose() : endPose);
    //follower.setPose(new Pose(EndPose.lastX, EndPose.lastY, EndPose.lastHeading));
    follower = Constants.createFollower(hardwareMap);
    follower.setStartingPose(EndPose.endPose);
    follower.update();
    telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
    leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
    rightFront = hardwareMap.get(DcMotor.class, "rightFront");
    rightBack  = hardwareMap.get(DcMotor.class, "rightBack");

    leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
    leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
    rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
    leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
    //set hardware map names (aka what the controller understands)
    intake = hardwareMap.get(DcMotorEx.class, "intake");
    transfer = hardwareMap.get(DcMotorEx.class, "transfer");

    intake.setDirection(DcMotorSimple.Direction.FORWARD);
    transfer.setDirection(DcMotorSimple.Direction.REVERSE);

    //0.44 low limit
    //0.1 high limit
    leftOuttake = hardwareMap.get(DcMotorEx.class, "leftOuttake");
    rightOuttake = hardwareMap.get(DcMotorEx.class, "rightOuttake");

    leftOuttake.setDirection(DcMotorSimple.Direction.REVERSE);
    rightOuttake.setDirection(DcMotorSimple.Direction.FORWARD);

    leftOuttake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightOuttake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    servo = hardwareMap.get(Servo.class, "axon");
    servo2 = hardwareMap.get(Servo.class, "axon2");
    servo.setDirection(Servo.Direction.REVERSE);
    pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
            .addPath(new Path(new BezierLine(follower::getPose, new Pose(follower.getPose().getX() + 0.001, follower.getPose().getY() + 0.001))))
            .setHeadingInterpolation(
                    HeadingInterpolator.lazy(() -> {
                        double robotX = follower.getPose().getX();
                        double robotY = follower.getPose().getY();
                        double alignX = targetX - robotX;
                        double alignY = targetY - robotY;
                        double angle = Math.atan2(alignY,alignX);
                        return HeadingInterpolator.linear(follower.getHeading(), angle);
                    })
            )
            .build();
}
    @Override
    public void start() {
        follower.startTeleopDrive(true);
    }
    @Override
    public void loop() {
        follower.update();
        Drive();
        shootTest();
    }
    private void Drive() {
        double slowModeMultiplier = 1 - (0.6 * gamepad1.left_trigger);
        follower.update();
        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors
            //This is the normal version to use in the TeleOp
            follower.setTeleOpDrive(
                    Math.pow(-gamepad1.left_stick_y,3) * slowModeMultiplier,
                    Math.pow(-gamepad1.left_stick_x,3) * slowModeMultiplier,
                    Math.pow(-gamepad1.right_stick_x,3) * slowModeMultiplier,
                    true // Robot Centric
            );
        }

        //Automated PathFollowing
        if (gamepad1.a) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }
        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }


    }



    public void shootTest() {
    /*
        servo.setPosition(servoPos);
        servo2.setPosition(servoPos);
        leftOuttake.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coeffs);
        rightOuttake.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coeffs);
        leftOuttake.setVelocity(ticksPerSecond);
        rightOuttake.setVelocity(ticksPerSecond);
        */
        boolean aPressed = gamepad1.left_bumper;

        if (aPressed && !lastA) {
            if (mode == shooterStates.CLOSE) {
                mode = shooterStates.FAR;
            } else {
                mode = shooterStates.CLOSE;
            }
        }

        lastA = aPressed;

        switch (mode) {
            case CLOSE:
                ticksPerSecond = 1200;
                servoPos = 0.125;
                minimum = 0;

                servo.setPosition(servoPos);
                servo2.setPosition((servoPos));
                leftOuttake.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coeffs);
                rightOuttake.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coeffs);
                leftOuttake.setVelocity(ticksPerSecond);
                rightOuttake.setVelocity(ticksPerSecond);
                break;

            case FAR:
                ticksPerSecond = 1560;
                servoPos = 0.075;
                minimum = 1520;

                servo.setPosition(servoPos);
                servo2.setPosition((servoPos));
                leftOuttake.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coeffs);
                rightOuttake.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coeffs);
                leftOuttake.setVelocity(ticksPerSecond);
                rightOuttake.setVelocity(ticksPerSecond);
                break;
        }


        /*
        if (gamepad1.b){
            ticksPerSecond = 200;
        } else if (gamepad1.a){
            ticksPerSecond = 900;
        } else {
            ticksPerSecond = 1200;
        }
        if (gamepad1.left_bumper) {
            servoPos = 0.5;
        } else {
            servoPos = 0.35;
        }*/
        double intakePower = 1;
        if (gamepad1.right_trigger > 0.15) {
            intake.setPower(intakePower);
        } else if (gamepad1.x) {
            intake.setPower(-intakePower);
        } else {
            intake.setPower(0);
        }





        if (gamepad1.right_bumper && leftOuttake.getVelocity()>minimum) {
            transferTime.reset();
            transfer.setPower(transferPower);

        } else if (gamepad1.y) {
            transfer.setPower(-transferPower);
        } else {
            transfer.setPower(0);
        }
        telemetry.addData("Ticks/s", ticksPerSecond);
        telemetry.addData("Left Velocity", leftOuttake.getVelocity());
        telemetry.addData("Right Velocity", rightOuttake.getVelocity());
        telemetry.addData("Error", ticksPerSecond-leftOuttake.getVelocity());
        telemetry.update();
    }
}
