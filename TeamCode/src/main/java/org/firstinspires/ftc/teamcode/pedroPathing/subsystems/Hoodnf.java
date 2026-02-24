package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

public class Hoodnf implements Subsystem {
    public static final Hoodnf INSTANCE = new Hoodnf();
    private Hoodnf() {}

    private final ServoEx variableHood = new ServoEx("Axon",-0.1);
    //------- The more the hood position, the more the ball arc, aka ball goes higher -------\\

    public Command closeSide() {
        return new SetPosition(variableHood, 0.32).requires(this);
    }

    public Command farSide() {
        return new SetPosition(variableHood, 0.38).requires(this);
    }

    public Command setHoodPos(double hoodPosition) {
        return new SetPosition(variableHood, hoodPosition).requires(this);
    }

}
