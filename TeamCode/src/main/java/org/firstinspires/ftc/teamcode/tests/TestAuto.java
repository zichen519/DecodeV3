package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.utils.Data;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;


import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
@Autonomous
public class TestAuto extends NextFTCOpMode {
    public TestAuto(){
        addComponents(
                new SubsystemComponent(),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }
    TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    private final Pose startPose = new Pose(72,72  , Math.toRadians(90));
    private final Pose leavePose = new Pose(72,100, Math.toRadians(90));


    private PathChain move;

    public void buildPaths() {
        move = PedroComponent.follower().pathBuilder().addPath(new BezierLine(startPose, leavePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), leavePose.getHeading())
                .build();

    }
    private Command auto(){
        return new FollowPath(move, true);

    }
    @Override
    public void onInit(){
        panelsTelemetry.addLine("Initialized");
        panelsTelemetry.update(telemetry);

        PedroComponent.follower().setStartingPose(startPose);
        buildPaths();
    }
    @Override
    public void onStartButtonPressed(){
        panelsTelemetry.addLine("Started");
        panelsTelemetry.update(telemetry);

        auto().schedule();
    }
    @Override
    public void onUpdate(){
        panelsTelemetry.addLine(CommandManager.INSTANCE.snapshot().toString());
        Data.endPose = PedroComponent.follower().getPose();
        telemetry.addData("Pose", Data.endPose.getX() + ", "+ Data.endPose.getY()+ ", "+ Data.endPose.getHeading());

        panelsTelemetry.update(telemetry);
    }

    @Override
    public void onStop(){

        Intake.INSTANCE.stopIntake.schedule();
        Shooter.INSTANCE.stopFlywheel.schedule();
    }
}
