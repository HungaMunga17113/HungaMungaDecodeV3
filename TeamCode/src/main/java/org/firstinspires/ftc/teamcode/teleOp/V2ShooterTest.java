package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
@Disabled
public class V2ShooterTest extends OpMode {

    // Initializing/making motor names
    DcMotor leftFront, leftBack, rightFront, rightBack;
    DcMotorEx leftOuttake, rightOuttake;
    //Initialize Variables
    private boolean returns = false;
    private long shootStartTime = 0;

    /*
    (Button) Initialize Period, before you press start on your program.
     */
    public void init() {

        //set hardware map names (aka what the controller understands)
        //leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        //leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        //rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        //rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        leftOuttake = hardwareMap.get(DcMotorEx.class, "leftOuttake");
        rightOuttake = hardwareMap.get(DcMotorEx.class, "rightOuttake");

        leftOuttake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightOuttake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftOuttake.setDirection(DcMotorSimple.Direction.REVERSE);
        rightOuttake.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    /*
    (Button) What happens when you press Start/Play
    You constantly update all your code
    Basically just keep all your code over here
     */
    public void loop() {

        shootTest();
    }

    /*
    Left Joystick ---------
        1. Up/Down - Robot goes forward, and backwards | all motors maxSpeed power, all motors -maxSpeed power
        2. Left/Right:
            a. Robot strafes left | (rightFront, leftBack maxSpeed) & (rightBack, leftFront -maxSpeed)
            b. Robot strafes right | (rightFront, leftBack -maxSpeed) & (rightBack, leftFront maxSpeed)

    Right Joystick ---------
        3. Turn:
            a. Robot strafes left | (rightFront, leftBack maxSpeed) & (rightBack, leftFront -maxSpeed)
            b. Robot strafes left | (rightFront, leftBack -maxSpeed) & (rightBack, leftFront maxSpeed)
     */

    public void shootTest() {
        if (gamepad1.right_trigger > 0.15) {
            leftOuttake.setPower(1);
            rightOuttake.setPower(1);
        }
        else {
            leftOuttake.setPower(0);
            rightOuttake.setPower(0);
        }
    }
    public void returnTest() {
        if (gamepad1.b && !returns) {
            returns=true;
            shootStartTime = System.currentTimeMillis();
            leftOuttake.setPower(-1);
            rightOuttake.setPower(-1);
        }
        long returnsDuration = 5000;
        if (returns) {
            if (System.currentTimeMillis() - shootStartTime >= returnsDuration) {
                returns=false;
                leftOuttake.setVelocity(0);
                rightOuttake.setVelocity(0);
            }
        }
    }
}







