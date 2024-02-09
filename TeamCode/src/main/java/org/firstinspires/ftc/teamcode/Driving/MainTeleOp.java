package org.firstinspires.ftc.teamcode.Driving;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.IMUUtils;
import org.firstinspires.ftc.teamcode.SimpleTeleSample.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.mathUtils.AngleUtils;
import org.firstinspires.ftc.teamcode.subsystems.DriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.LiftCommand;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

@TeleOp(name = "Main Tele Op")
public class MainTeleOp extends CommandOpMode {
    public static double heading = 0;
    public static double normalHeading = 0;
    private double correction = 0;
    private IMUUtils imu = new IMUUtils();

    @Override
    public void initialize() {
        GamepadEx driver = new GamepadEx(gamepad1);
        GamepadEx operator = new GamepadEx(gamepad2);
        imu.init(hardwareMap, this);

        schedule(new RunCommand(()->{
            heading = imu.getAngle();
            normalHeading = AngleUtils.convertToNormalAngle(heading);
            correction = AngleUtils.checkCorrection(operator.getButton(GamepadKeys.Button.A),
                    normalHeading);
        }));

        DriveSubsystem driveSystem = new DriveSubsystem(
                new Motor(hardwareMap, "FL"),
                new Motor(hardwareMap, "FR"),
                new Motor(hardwareMap, "BL"),
                new Motor(hardwareMap, "BR"),
                driver, telemetry
        );

        LiftSubsystem liftSubsystem = new LiftSubsystem(new Motor(hardwareMap, "lift"),
                operator, telemetry);

        IntakeSubsystem intakeSubsystem = new IntakeSubsystem(
                new Motor(hardwareMap, "intake"),
                operator,
                telemetry
        );


        driveSystem.setDefaultCommand(new DriveCommand(driveSystem));
        liftSubsystem.setDefaultCommand(new LiftCommand(liftSubsystem));
        intakeSubsystem.setDefaultCommand(new IntakeCommand(intakeSubsystem));

        schedule(new RunCommand(telemetry::update));

    }
}
