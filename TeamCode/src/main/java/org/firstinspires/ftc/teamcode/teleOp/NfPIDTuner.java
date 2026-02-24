package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Hoodnf;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Intakenf;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Shooternf;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Transfernf;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Disabled
@TeleOp
public class NfPIDTuner extends NextFTCOpMode {

    public NfPIDTuner() {
        addComponents(
                new SubsystemComponent(Hoodnf.INSTANCE, Shooternf.INSTANCE, Transfernf.INSTANCE, Intakenf.INSTANCE),
                BulkReadComponent.INSTANCE,

                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onStartButtonPressed() {
        Shooternf.INSTANCE.close();
        Gamepads.gamepad1().rightBumper().whenBecomesTrue(
                Transfernf.INSTANCE.out()
        );
        Gamepads.gamepad1().rightBumper().whenBecomesFalse(
                Transfernf.INSTANCE.idle()
        );
        Gamepads.gamepad1().rightTrigger().greaterThan(0.2).whenBecomesTrue(
                Intakenf.INSTANCE.in()
        );
        Gamepads.gamepad1().rightTrigger().lessThan(0.2).whenBecomesTrue(
                Intakenf.INSTANCE.idle()
        );
    }
}
