package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Driving.MainTeleOp.heading;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class LiftSubsystem extends SubsystemBase {
    private DoubleSupplier liftSpeed;
    private Motor lift;
    private Telemetry telemetry;
    public static double speedMultiplier = 0.65;

    public LiftSubsystem(Motor lift,
                         GamepadEx gamepad,
                         Telemetry telemetry) {
        liftSpeed = gamepad::getRightY;
        this.lift = lift;
        this.telemetry = telemetry;
    }

    public void operate() {
        lift.set(liftSpeed.getAsDouble());
    }
}