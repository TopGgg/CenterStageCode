package org.firstinspires.ftc.teamcode.auto;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.IMUUtils;
import org.firstinspires.ftc.teamcode.mathUtils.AngleUtils;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

@Autonomous(name = "Main Auto", group = "Auto", preselectTeleOp = "Main Tele Op")
public class MainAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        IMUUtils imu = new IMUUtils(null);
        imu.init(hardwareMap,this);
        Robot robot = new Robot(new Motor(hardwareMap, "FL"),
                new Motor(hardwareMap, "FR"),
                new Motor(hardwareMap, "BL"),
                new Motor(hardwareMap, "BR"),
                imu::getAngle,
                this);
        LiftSubsystem liftSubsystem = new LiftSubsystem(new Motor(hardwareMap, "lift"),
                hardwareMap.servo.get("stick"), hardwareMap.servo.get("rotation"),
                hardwareMap.servo.get("gripper"),
                hardwareMap.get(DistanceSensor.class, "grip"),
                null, telemetry);

        waitForStart();
        robot.forward(2000, -1);
        robot.strafe(700, -1);
        liftSubsystem.setGripperState(true);
        liftSubsystem.lift.set(0.5);
        Thread.sleep(300);
        liftSubsystem.lift.set(0);
        liftSubsystem.setStickState(LiftSubsystem.StickState.BOARD);
        liftSubsystem.setRotationState(LiftSubsystem.StickState.BOARD);
        liftSubsystem.lift.set(0.5);
        Thread.sleep(300);
        liftSubsystem.lift.set(0);
        robot.forward(1000, -1);
        liftSubsystem.setGripperState(false);

    }
}
