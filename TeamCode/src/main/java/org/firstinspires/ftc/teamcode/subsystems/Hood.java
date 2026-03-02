package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.utils.Calculations;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.positionable.SetPositions;

public class Hood implements Subsystem {
    public static final Hood INSTANCE = new Hood();
    private Hood(){}

    private ServoEx hood = new ServoEx("hood");

    public Command up = new SetPosition(hood,0.15).requires(this);
    public Command down = new SetPosition(hood,0.62).requires(this);
    public Command close = new SetPosition(hood,0.42).requires(this);
    public Command calculateAngle(double distance){
        return new SetPosition(hood, .62-Calculations.getHoodAngle(distance)).requires(this);
    }


    @Override
    public void initialize(){

    }

    @Override
    public void periodic(){

    }
}
