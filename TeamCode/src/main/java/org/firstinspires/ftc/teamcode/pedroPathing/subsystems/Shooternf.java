package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.bylazar.configurables.annotations.Configurable;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;

@Configurable
public class Shooternf implements Subsystem {
    public static final Shooternf INSTANCE = new Shooternf();
    private Shooternf() { }

    public MotorEx leftOuttake, rightOuttake;
    public MotorGroup shooter;

    private final ControlSystem closeShooterController = ControlSystem.builder()
            .velPid(2.14, 0, 0.003)
            .basicFF(0.002)
            .build();

    private final ControlSystem farShooterController = ControlSystem.builder()
            .velPid(2.47, 0, 0.003)
            .basicFF(0.006)
            .build();

    private boolean enabled = false;

    private enum ShooterControllerMode {
        CLOSE,
        FAR
    }

    private ShooterControllerMode currentControllerMode = ShooterControllerMode.CLOSE;


    public Command close() {
        currentControllerMode = ShooterControllerMode.CLOSE;
        return new RunToVelocity(closeShooterController, -1205).requires(shooter);
    }

    public Command far() {
        currentControllerMode = ShooterControllerMode.FAR;
        return new RunToVelocity(farShooterController, -1540).requires(shooter);
    }

    public Command idle() {
        currentControllerMode = ShooterControllerMode.CLOSE;
        return new RunToVelocity(closeShooterController, 0).requires(shooter);
    }

    public Command setShooterVel(double shooterVel) {
        currentControllerMode = ShooterControllerMode.CLOSE;
        return new RunToVelocity(closeShooterController, shooterVel).requires(shooter);
    }

    public Command setShooterVel(double shooterVel, boolean farSide) {
        if (farSide) {
            currentControllerMode = ShooterControllerMode.FAR;
        } else {
            currentControllerMode = ShooterControllerMode.CLOSE;
        }

        return new RunToVelocity(
                farSide ? farShooterController : closeShooterController,
                shooterVel
        ).requires(shooter);
    }

    public void enable() {
        enabled = true;
    }

    public void disable() {
        enabled = false;
        shooter.setPower(0);
    }

    @Override
    public void initialize() {
        leftOuttake = new MotorEx("leftOuttake");
        rightOuttake = new MotorEx("rightOuttake");
        rightOuttake.reverse();
        shooter = new MotorGroup(leftOuttake, rightOuttake);

        disable();
    }

    @Override
    public void periodic() {
        if (!enabled) {
            shooter.setPower(0);
            return;
        }

        ControlSystem controller;

        if (currentControllerMode == ShooterControllerMode.FAR) {
            controller = farShooterController;
        } else {
            controller = closeShooterController;
        }

        shooter.setPower(controller.calculate(shooter.getState()));
    }
}