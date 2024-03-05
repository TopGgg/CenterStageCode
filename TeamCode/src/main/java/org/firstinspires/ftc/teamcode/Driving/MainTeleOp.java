package org.firstinspires.ftc.teamcode.Driving;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.AutoPlay.AutoPlay;
import org.firstinspires.ftc.teamcode.AutoPlay.BoardDetection;
import org.firstinspires.ftc.teamcode.AutoPlay.BoardDetectionPipeline;
import org.firstinspires.ftc.teamcode.AutoPlay.CameraAligner;
import org.firstinspires.ftc.teamcode.IMUUtils;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
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
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        GamepadEx driver = new GamepadEx(gamepad1);
        GamepadEx operator = new GamepadEx(gamepad2);
        imu.init(hardwareMap, this);

        schedule(new RunCommand(()->{
            heading = imu.getAngle();
            normalHeading = AngleUtils.convertToNormalAngle(heading);
            correction = AngleUtils.checkCorrection(operator.getButton(GamepadKeys.Button.A),
                    normalHeading);
            operator.readButtons();
        }));
        DriveSubsystem driveSystem = new DriveSubsystem(
                new Motor(hardwareMap, "FL"),
                new Motor(hardwareMap, "FR"),
                new Motor(hardwareMap, "BL"),
                new Motor(hardwareMap, "BR"),
                driver, telemetry
        );

        LiftSubsystem liftSubsystem = new LiftSubsystem(new Motor(hardwareMap, "lift"),
                hardwareMap.servo.get("stick"), hardwareMap.servo.get("rotation"),
                hardwareMap.servo.get("gripper"),
                operator, telemetry);

        IntakeSubsystem intakeSubsystem = new IntakeSubsystem(
                new Motor(hardwareMap, "intake"),
                operator,
                telemetry
        );

        AutoPlay autoPlay = new AutoPlay(this,hardwareMap,telemetry,operator);

//        CameraAligner aligner = new CameraAligner(hardwareMap.servo.get("cameraServo"));
//
//        aligner.setDefaultCommand(new CameraAligner.CameraAlignerCommand(aligner));
        driveSystem.setDefaultCommand(new DriveCommand(driveSystem));
        liftSubsystem.setDefaultCommand(new LiftCommand(liftSubsystem));
        intakeSubsystem.setDefaultCommand(new IntakeCommand(intakeSubsystem));
        autoPlay.setDefaultCommand(new AutoPlay.AutoPlayCommand(autoPlay));

        schedule(new RunCommand(()->{
            telemetry.addData("heading", normalHeading);
            if(driver.getButton(GamepadKeys.Button.A)){
                imu.init(hardwareMap,MainTeleOp.this);
            }
            telemetry.addData("score", AutoPlay.score);
            telemetry.addData("action 1", AutoPlay.action1);
            telemetry.addData("action 2", AutoPlay.action2);
            telemetry.addData("action 3", AutoPlay.action3);
            telemetry.addData("AutoPlay Active", BoardDetectionPipeline.active);
        }));
        schedule(new RunCommand(telemetry::update));

    }
}
