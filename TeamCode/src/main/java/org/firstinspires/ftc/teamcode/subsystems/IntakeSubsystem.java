package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.BooleanSupplier;

public class IntakeSubsystem extends SubsystemBase {
    public Motor intake;
    public GamepadEx operator;
    public Telemetry telemetry;
    public BooleanSupplier forward;
    public BooleanSupplier backward;
    private static final double speed = 1;
    public IntakeSubsystem(Motor intake,
                           GamepadEx operator,
                           Telemetry telemetry){
        this.intake = intake;
        this.operator = operator;
        this.telemetry = telemetry;
        forward = () -> operator.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0;
        backward = () -> operator.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0;
    }

    public void operate(){
        if(backward.getAsBoolean()){
            reverse();
        }else if(forward.getAsBoolean()){
            activate();
        }else {
            stop();
        }
    }

    public void activate(){
        intake.set(speed);
    }

    public void reverse(){
        intake.set(-speed);
    }

    public void stop(){
        intake.stopMotor();
    }
}
