package org.firstinspires.ftc.teamcode.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
@Configurable
@TeleOp

public class ShooterPIDTest extends OpMode {

    DcMotorEx leftOuttake, rightOuttake,intake,transfer;
    public static double ticksPerSecond = 1250;
    //1500 is far
    //1250 is close
    public static double servoPos = 0.393;
    //0.38 is far
    //0.393 is close
    public double minimum = 0;
    //0 is close
    //1480 is far
    public Servo servo;

    public static double transferPower = 1;
    public static PIDFCoefficients coeffs = new PIDFCoefficients(329, 0.00035, 0.01, 18.4);

    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        leftOuttake = hardwareMap.get(DcMotorEx.class, "leftOuttake");
        rightOuttake = hardwareMap.get(DcMotorEx.class, "rightOuttake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        leftOuttake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightOuttake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftOuttake.setDirection(DcMotorSimple.Direction.REVERSE);
        rightOuttake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);

        servo = hardwareMap.get(Servo.class, "Axon");


    }

    public void loop() {

        shootTest();

    }

    public void shootTest() {
        servo.setPosition(servoPos);
        leftOuttake.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coeffs);
        rightOuttake.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coeffs);
        leftOuttake.setVelocity(ticksPerSecond);
        rightOuttake.setVelocity(ticksPerSecond);

        /*
        leftOuttake.setPower(outtakePower);
        rightOuttake.setPower(outtakePower);
        */
        double intakePower = 1;

        if (gamepad1.right_bumper) {
            intake.setPower(intakePower);
        } else if (gamepad1.y) {
            intake.setPower(-intakePower);
        } else {
            intake.setPower(0);
        }
        /*
        if (ticksPerSecond<1350) {
            minimum = 0;
            maximum = 1330;
        } else {
            minimum = 1475;
            maximum = 1575;
        }

         */

        if (gamepad1.right_trigger > 0.15 && leftOuttake.getVelocity()>minimum) {
                transfer.setPower(transferPower);
        } else if (gamepad1.x) {
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

