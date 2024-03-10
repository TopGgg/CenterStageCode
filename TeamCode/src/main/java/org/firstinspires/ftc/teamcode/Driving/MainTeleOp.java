package org.firstinspires.ftc.teamcode.Driving;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.sun.tools.javac.jvm.Gen;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.AutoPlay.AutoPlay;
import org.firstinspires.ftc.teamcode.AutoPlay.BoardDetection;
import org.firstinspires.ftc.teamcode.AutoPlay.BoardDetectionPipeline;
import org.firstinspires.ftc.teamcode.AutoPlay.CameraAligner;
import org.firstinspires.ftc.teamcode.IMUUtils;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.AntiSlip;
import org.firstinspires.ftc.teamcode.subsystems.AutoGrip;
import org.firstinspires.ftc.teamcode.subsystems.AutoIntake;
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
    public double correction;
    public static boolean isFalling;
    public static boolean isFallingBack;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        GamepadEx driver = new GamepadEx(gamepad1);
        GamepadEx operator = new GamepadEx(gamepad2);
        SampleMecanumDrive odometry = new SampleMecanumDrive(hardwareMap, false);
        IMUUtils imu = new IMUUtils(odometry.imu);
//        imu.init(hardwareMap, this);

        schedule(new RunCommand(()->{
            heading = imu.getAngle();
            normalHeading = AngleUtils.convertToNormalAngle(heading);
            correction = AngleUtils.checkCorrection(operator.getButton(GamepadKeys.Button.A),
                    normalHeading);
            operator.readButtons();
            odometry.update();

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
                hardwareMap.get(DistanceSensor.class, "grip"),
                operator, telemetry);

        IntakeSubsystem intakeSubsystem = new IntakeSubsystem(
                new Motor(hardwareMap, "intake"),
                operator,
                telemetry
        );

        AutoPlay autoPlay = new AutoPlay(this,hardwareMap,telemetry,operator);
        AutoGrip autoGrip = new AutoGrip(
                hardwareMap.get(DistanceSensor.class, "distSensor"),
                hardwareMap.servo.get("locker"),liftSubsystem,
                ()->operator.wasJustPressed(GamepadKeys.Button.DPAD_LEFT));
        AutoIntake autoIntake = new AutoIntake(odometry, intakeSubsystem,
                ()->operator.wasJustPressed(GamepadKeys.Button.DPAD_UP));
        AntiSlip antiSlip = new AntiSlip();

//        CameraAligner aligner = new CameraAligner(hardwareMap.servo.get("cameraServo"));
//
//        aligner.setDefaultCommand(new CameraAligner.CameraAlignerCommand(aligner));
        driveSystem.setDefaultCommand(new DriveCommand(driveSystem));
        liftSubsystem.setDefaultCommand(new LiftCommand(liftSubsystem));
        intakeSubsystem.setDefaultCommand(new IntakeCommand(intakeSubsystem));
        autoPlay.setDefaultCommand(new AutoPlay.AutoPlayCommand(autoPlay));
        autoGrip.setDefaultCommand(new AutoGrip.AutoGripCommand(autoGrip));
        autoIntake.setDefaultCommand(new AutoIntake.AutoIntakeCommand(autoIntake));
        antiSlip.setDefaultCommand(new AntiSlip.AntiSlipCommand(antiSlip));

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
            telemetry.addData("AutoGrip Active", AutoGrip.active);
            telemetry.addData("AutoIntake Active", AutoIntake.active);
        }));
        schedule(new RunCommand(telemetry::update));

    }
}
