package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.utils.Calculations;
import org.firstinspires.ftc.teamcode.utils.Data;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import java.util.List;

import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp
public class TeleopRed extends NextFTCOpMode {
    public TeleopRed(){
        addComponents(
                new SubsystemComponent(Shooter.INSTANCE, Intake.INSTANCE, Transfer.INSTANCE, Turret.INSTANCE, Hood.INSTANCE, Limelight.INSTANCE, Lift.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );

    }
    //public static Pose endPose;
    private MotorEx frontLeftMotor;
    private MotorEx frontRightMotor;
    private MotorEx backLeftMotor;
    private MotorEx backRightMotor;

    private boolean runShooter = false;
    public double transferPower = 0.5;

    public static Follower getFollower(){
        return PedroComponent.follower();
    }
    @Override
    public void onStartButtonPressed() {
        DriverControlledCommand driverControlled = new PedroDriverControlled(
                Gamepads.gamepad1().rightStickY().negate(),
                Gamepads.gamepad1().rightStickX().negate(),
                Gamepads.gamepad1().leftStickX().map(value -> 0.5*value).negate()
        );
        driverControlled.schedule();
        Lift.INSTANCE.holdPlate().schedule();
        Gamepads.gamepad1().circle().whenBecomesTrue(Intake.INSTANCE.runIntake);
        Gamepads.gamepad1().square().whenBecomesTrue(Intake.INSTANCE.runIntakeReverse);
        Gamepads.gamepad1().dpadRight().whenBecomesTrue(Intake.INSTANCE.stopIntake);
        Gamepads.gamepad1().leftBumper().whenBecomesTrue(Transfer.INSTANCE.runTransfer(0.4)).whenBecomesFalse(Transfer.INSTANCE.runTransfer(0.0));

        Gamepads.gamepad1().rightBumper().whenBecomesTrue(Transfer.INSTANCE.runTransfer(0.7)).whenBecomesFalse(Transfer.INSTANCE.runTransfer(0.0));
        //Gamepads.gamepad1().dpadUp().whenBecomesTrue(Shooter.INSTANCE.runFlywheelClose.and(Hood.INSTANCE.close));
        //Gamepads.gamepad1().dpadDown().whenBecomesTrue(Shooter.INSTANCE.runFlywheelFar.and(Hood.INSTANCE.up));
        Gamepads.gamepad1().rightBumper().whenBecomesTrue(Transfer.INSTANCE.runTransfer(1.0)).whenBecomesFalse(Transfer.INSTANCE.runTransfer(0.0));
        Gamepads.gamepad2().circle().whenBecomesTrue(Lift.INSTANCE.lift()).whenBecomesFalse(Lift.INSTANCE.holdLift());
    }
    @Override
    public void onInit(){
        CommandManager.INSTANCE.cancelAll();

        frontLeftMotor = new MotorEx("FL").reversed();
        frontRightMotor = new MotorEx("FR");
        backLeftMotor = new MotorEx("BL").reversed();
        backRightMotor = new MotorEx("BR");
        if(Data.endPose!=null){
            telemetry.addData("endpose", Data.endPose.getX() + ", "+ Data.endPose.getY()+ ", "+ Data.endPose.getHeading());
            PedroComponent.follower().setStartingPose(Data.endPose);
        }
        else PedroComponent.follower().setPose(new Pose(72,72, Math.toRadians(90)));
        Turret.alignment=true;
        Calculations.goalPose=Calculations.redGoalPose;
        telemetry.addData("curr pose", PedroComponent.follower().getPose().getX() + ", "+ PedroComponent.follower().getPose().getY()+ ", "+ PedroComponent.follower().getPose().getHeading());

        //telemetry.addData("X",PedroComponent.follower().getPose().getX());
        //telemetry.addData("Y",PedroComponent.follower().getPose().getY());
        telemetry.update();
        Limelight.INSTANCE.limelight.pipelineSwitch(1);
    }
    @Override
    public void onUpdate(){
        LLResult result = Limelight.INSTANCE.getLatestResult();
        if(gamepad1.dpad_up) {
            runShooter = true;
            Turret.alignment = true;
        }
        if(gamepad1.dpad_down) {
            runShooter = false;
            Turret.alignment = false;
        }

        //if(gamepad1.right_bumper) Transfer.INSTANCE.runTransfer(transferPower).schedule();
        //else Transfer.INSTANCE.runTransfer(0.0).schedule();
        if(result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                double ty = fr.getTargetYDegrees();
                double distance = 16 / (Math.tan(Math.toRadians(7.6026 + ty)));

                ActiveOpMode.telemetry().addData("ty", ty);
                ActiveOpMode.telemetry().addData("dist", distance);
                ActiveOpMode.telemetry().addData("target rpm", Calculations.getShooterRPM(distance));
                ActiveOpMode.telemetry().addData("hood offset", .63-Calculations.getHoodAngle(distance));
                if(runShooter){
                    Shooter.INSTANCE.calculateFlywheel(distance).schedule();
                    Hood.INSTANCE.calculateAngle(distance).schedule();
                }
                else{
                    Shooter.INSTANCE.stopFlywheel.schedule();
                }


                //transferPower = Calculations.getTransferSpeed(distance);

                //ActiveOpMode.telemetry().addData("transfer power", transferPower);


            }
        }
        //telemetry.addData("distance", Math.hypot(Calculations.redGoalPose.getY()-PedroComponent.follower().getPose().getY(),Calculations.redGoalPose.getX()-PedroComponent.follower().getPose().getX()));
        telemetry.addData("x y heading", PedroComponent.follower().getPose().getX() + ", "+ PedroComponent.follower().getPose().getY()+ ", "+ PedroComponent.follower().getPose().getHeading());
        telemetry.addData("Commands", CommandManager.INSTANCE.snapshot());
        telemetry.update();
    }
    @Override
    public void onStop(){
        Turret.alignment=false;
        CommandManager.INSTANCE.cancelAll();
        Shooter.INSTANCE.stopFlywheel.schedule();
        Data.endPose=PedroComponent.follower().getPose();
    }
}
