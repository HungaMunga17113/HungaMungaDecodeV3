package org.firstinspires.ftc.teamcode.pedroPathing;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Hoodnf;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Intakenf;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Shooternf;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Transfernf;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

//
@Disabled
@Autonomous
@Configurable
public class GuptaGreatCurryMuncher extends NextFTCOpMode {
    public GuptaGreatCurryMuncher() {
        addComponents(
                new SubsystemComponent(
                        Intakenf.INSTANCE, Hoodnf.INSTANCE,
                        Shooternf.INSTANCE, Transfernf.INSTANCE
                ),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }


    public PathChain Shoot1;
    public PathChain Intake1;
    public PathChain Shoot2;
    public PathChain Intake2;
    public PathChain Gate;
    public PathChain Shoot3;
    public PathChain Intake4;
    public PathChain Shoot4;
    public PathChain Intake5;
    public PathChain Shoot6;

    public void paths() {
        PedroComponent.follower().setStartingPose(new Pose(25.147, 127.393, Math.toRadians(143)));


        Shoot1 = PedroComponent.follower()
                .pathBuilder().addPath(
                        new BezierLine(
                                new Pose(25.147, 127.393),

                                new Pose(48.100, 95.545)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(135))
                .build();

        Intake1 = PedroComponent.follower()
                .pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(48.100, 95.545),
                                new Pose(40.329, 84.491),
                                new Pose(16.301, 84.491)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(135),Math.toRadians(180))

                .build();

        Shoot2 = PedroComponent.follower()
                .pathBuilder().addPath(
                        new BezierLine(
                                new Pose(16.301, 84.491),

                                new Pose(48.171, 95.400)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                .build();

        Intake2 = PedroComponent.follower()
                .pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(48.171, 95.400),
                                new Pose(45.488, 56.939),
                                new Pose(32.117, 56.939),
                                new Pose(11.573, 56.939)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                .build();

        Gate = PedroComponent.follower()
                .pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(11.573, 56.939),
                                new Pose(34.433, 56.939),
                                new Pose(17.318, 69.509)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Shoot3 = PedroComponent.follower()
                .pathBuilder().addPath(
                        new BezierLine(
                                new Pose(17.318, 69.509),

                                new Pose(48.171, 95.545)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                .build();

        Intake4 = PedroComponent.follower()
                .pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(48.171, 95.545),
                                new Pose(61.761, 34.570),
                                new Pose(10.831, 34.570)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        Shoot4 = PedroComponent.follower()
                .pathBuilder().addPath(
                        new BezierLine(
                                new Pose(10.831, 34.570),
                                new Pose(48.171, 95.545)
                        )
                )
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.8,
                                        HeadingInterpolator.tangent.reverse()
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.8,
                                        1.0,
                                        HeadingInterpolator.facingPoint(new Pose(0,144))
                                )
                        )
                )
                .build();

        Intake5 = PedroComponent.follower()
                .pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(48.171, 95.545),
                                new Pose(43.998, 24.869),
                                new Pose(12.863, 13.206)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        Shoot6 = PedroComponent.follower()
                .pathBuilder().addPath(
                        new BezierLine(
                                new Pose(12.863, 13.206),

                                new Pose(56.794, 109.972)
                        )
                )
                .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                                new HeadingInterpolator.PiecewiseNode(
                                        0,
                                        0.8,
                                        HeadingInterpolator.tangent.reverse()
                                ),
                                new HeadingInterpolator.PiecewiseNode(
                                        0.8,
                                        1.0,
                                        HeadingInterpolator.facingPoint(new Pose(0,144))
                                )
                        )
                )

                .build();
    }



    private Command set_hood() {
        return new SequentialGroup(
                Hoodnf.INSTANCE.setHoodPos(0.41)
        );

    }

    private Command transferUpFor(double time) {
        return new SequentialGroup(
                Transfernf.INSTANCE.out(),
                new Delay(time),
                Transfernf.INSTANCE.idle()
        );
    }

    private Command baseState() {
        return new ParallelGroup(
                Transfernf.INSTANCE.idle(),
                Hoodnf.INSTANCE.setHoodPos(0.42)
        );
    }
    private Command autonomous() {
        return new ParallelGroup(
                //INTAKE ALWAYS ON
                Intakenf.INSTANCE.in(),
                Shooternf.INSTANCE.close(),
                //MAIN SEQUENCE
                new SequentialGroup(

                        //Preloads
                        new ParallelGroup(
                                new FollowPath(Shoot1, true),
                                baseState()
                        ),
                        new Delay(1),
                        transferUpFor(0.7),


                        //SET 2
                        new ParallelGroup(
                                new SequentialGroup(
                                        new FollowPath(Intake1),
                                        new FollowPath(Shoot2, true)
                                )
                        ),
                        new Delay(0.4),
                        transferUpFor(0.7),

                        //SET 3 Human Player
                        new ParallelGroup(
                                new SequentialGroup(
                                        new FollowPath(Intake2),
                                        new FollowPath(Gate),
                                        new Delay(0.4),
                                        new FollowPath(Shoot3, true)
                                )
                        ),
                        new Delay(0.4),
                        transferUpFor(0.7),
                        new ParallelGroup(
                                new SequentialGroup(
                                        new FollowPath(Intake4),
                                        new FollowPath(Shoot4, true)
                                )
                        ),
                        new Delay(0.4),
                        transferUpFor(0.7),
                        //SET 4
                        new SequentialGroup(
                                new SequentialGroup(
                                        new FollowPath(Intake5),
                                        new FollowPath(Shoot6, true)
                                ),
                                new Delay(0.4),
                                transferUpFor(0.75)
                        )


                )
        );
    }
    @Override
    public void onInit() {
        paths();
        set_hood().schedule();
        Shooternf.INSTANCE.disable();
    }
    @Override
    public void onStartButtonPressed() {
        Shooternf.INSTANCE.enable();

        autonomous().schedule();

    }
    @Override
    public void onStop() {
        EndPose.endPose = follower().getPose();
        EndPose.lastX = follower().getPose().getX();
        EndPose.lastY = follower().getPose().getY();
        EndPose.lastHeading = follower().getHeading();    }
}