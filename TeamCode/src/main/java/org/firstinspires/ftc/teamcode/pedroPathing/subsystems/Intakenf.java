package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class Intakenf implements Subsystem {
    public static final Intakenf INSTANCE = new Intakenf();
    private Intakenf() { }

    public MotorEx intake;

    public enum intakeStates {
        IN (1.0),
        IDLE (0),
        OUT (-1.0);

        public final double intakeState;
        intakeStates(double state) {
            this.intakeState = state;
        }
        public double getState() {
            return intakeState;
        }
    }

    public Command in() {
        return new SetPower(intake, intakeStates.IN.getState()).requires(intake);
    }
    public Command idle() {
        return new SetPower(intake, intakeStates.IDLE.getState()).requires(intake);
    }
    public Command out() {
        return new SetPower(intake, intakeStates.OUT.getState()).requires(intake);
    }

    @Override
    public void initialize() {
        intake = new MotorEx("intake");
    }

    @Override
    public void periodic() {}
}

