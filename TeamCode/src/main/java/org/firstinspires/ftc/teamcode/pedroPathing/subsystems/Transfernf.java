package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.pedropathing.paths.PathChain;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class Transfernf implements Subsystem {

    public static final Transfernf INSTANCE = new Transfernf();
    private Transfernf() {}

    public MotorEx transfer;

    public Command out() {
        return new SetPower(transfer, -0.92);
    }
    public Command outSlow() {
        return new SetPower(transfer, -0.6);
    }

    public Command farTransfer() {
        return new SequentialGroup(
                new SetPower(transfer, -0.6),
                new Delay(0.2),
                new SetPower(transfer, -0.6),
                new Delay(0.2),
                new SetPower(transfer, -0.6),
                new Delay(0.2)
        );
    }

    public Command stepOn() {
        return new SequentialGroup(
                new SetPower(transfer, -0.5),
                new Delay(0.2),
                new SetPower(transfer, -1.0)
        );
    }

    public Command stepOn(double maxPower) {
        return new SequentialGroup(
                new SetPower(transfer, -0.5),
                new Delay(0.2),
                new SetPower(transfer, -maxPower)
        );
    }

    public Command pickup(PathChain pathChain, double distanceForHotdog) {
        return new SequentialGroup(
                new SetPower(transfer, -0.4),
                new WaitUntil(() ->
                        pathChain.lastPath().getDistanceRemaining() < distanceForHotdog
                )
        );
    }

    public Command idle() {
        return new SetPower(transfer, 0);
    }

    public Command forceBackOn() {
        return new SetPower(transfer, -1.0);
    }

    @Override
    public void initialize() {
        transfer = new MotorEx("transfer");
    }

    @Override
    public void periodic() {}
}
