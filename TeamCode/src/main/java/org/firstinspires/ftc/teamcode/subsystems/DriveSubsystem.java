package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Driving.MainTeleOp.heading;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Driving.MainTeleOp;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DriveSubsystem extends SubsystemBase {
    private DoubleSupplier strafe, forward, rotation;
    private BooleanSupplier fast, slow;
    public MecanumDrive drive;
    public Motor FL,FR,BL,BR;
    private Telemetry telemetry;
    public static double speedMultiplier = 0.65;

    public DriveSubsystem(Motor FL, Motor FR,
                          Motor BL, Motor BR,
                          GamepadEx gamepad,
                          Telemetry telemetry) {
        strafe = gamepad::getLeftX;
        forward = gamepad::getLeftY;
        rotation = gamepad::getRightX;

        fast = () -> gamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER);
        slow = () -> gamepad.getButton(GamepadKeys.Button.LEFT_BUMPER);

        drive = new MecanumDrive(FL, FR, BL, BR);
        this.telemetry = telemetry;
        this.FL = FL;
        this.FR = FR;
        this.BL = BL;
        this.BR = BR;

    }

    public void drive() {
        if(fast.getAsBoolean() && slow.getAsBoolean()){
            speedMultiplier = 1;
        }else if(slow.getAsBoolean()){
            speedMultiplier = 0.35;
        }else if (fast.getAsBoolean()){
            speedMultiplier = 0.8;
        }else {
            speedMultiplier = 0.65;
        }

        telemetry.addData("multiplier", speedMultiplier);
        if(MainTeleOp.isFalling){
            drive.driveRobotCentric(0,MainTeleOp.isFallingBack ? -1 : 1, 0);
        }else{
            drive.driveFieldCentric(-strafe.getAsDouble()*speedMultiplier,
                    -forward.getAsDouble()*speedMultiplier,
                    -rotation.getAsDouble()*speedMultiplier, heading);
        }
    }
}