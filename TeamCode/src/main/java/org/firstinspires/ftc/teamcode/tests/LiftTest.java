package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.impl.FeedbackCRServoEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPositions;

@TeleOp
@Configurable
public class LiftTest extends NextFTCOpMode {
    public LiftTest(){
        addComponents(
                new SubsystemComponent(),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }
    public static double power = 0.0;

    private FeedbackCRServoEx liftLeft = new FeedbackCRServoEx(0.01,() -> ActiveOpMode.hardwareMap().analogInput.get("lift1"), () -> ActiveOpMode.hardwareMap().crservo.get("liftL"));

    private CRServoEx liftRight = new CRServoEx("liftR");
    double totalAngle = 0.0; // This is your angle of the servo
    double previousAngle = 0.0;
    //-38.5
    @Override
    public void onStartButtonPressed() {


        //Shooter.INSTANCE.runFlywheelMid.schedule();

    }

    @Override
    public void onUpdate(){
        double currentAngle = liftLeft.getCurrentPosition();
        double deltaAngle = currentAngle - previousAngle;

        if (deltaAngle > Math.PI) deltaAngle -= 2 * Math.PI;
        else if (deltaAngle < -Math.PI) deltaAngle += 2 * Math.PI;

        totalAngle += deltaAngle;
        previousAngle = currentAngle;
        liftLeft.setPower(power);
        liftRight.setPower(-power);

        telemetry.addData("angle",totalAngle);
        telemetry.update();
    }
}
