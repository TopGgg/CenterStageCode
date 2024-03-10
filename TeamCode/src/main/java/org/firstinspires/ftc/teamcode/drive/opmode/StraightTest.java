package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class StraightTest extends LinearOpMode {
    public static double DISTANCE = 60; // in

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        //can be either 1 2 or 3
        byte pixelPosition = 1;
        Servo pixel = hardwareMap.servo.get("propServo");
        pixel.setPosition(0.59);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        TrajectorySequence case2 = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(31.8)
                .addDisplacementMarker(()->pixel.setPosition(0))
                .back(10)
                .turn(Math.toRadians(97.5))
                .forward(80)
                .waitSeconds(3)
                .build();


        TrajectorySequence case1 = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(26)
                .turn(Math.toRadians(-90))
                .forward(9.5)
                .addDisplacementMarker(()->pixel.setPosition(0)) //pixel servo
                .back(9.5)
                .turn(Math.toRadians(90))
                .back(5)
                .turn(Math.toRadians(108.5))
                .forward(80)
                .addDisplacementMarker(()->{}) //lift
                .build();

        TrajectorySequence case3 = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(29)
                .turn(Math.toRadians(-108.5))
                .back(26)
                .waitSeconds(1)
                .addDisplacementMarker(()->pixel.setPosition(0)) //pixel servo
                .waitSeconds(1)
                .back(30)
                .turn(Math.toRadians(115.5*2))
                .forward(45)
                .addDisplacementMarker(()->{}) //lift
                .build();

        drive.followTrajectorySequence(case1);
//        if(pixelPosition == 1){
//            drive.followTrajectorySequence(case1);
//        } else if (pixelPosition == 2) {
//            drive.followTrajectorySequence(case2);
//        } else {
//            drive.followTrajectorySequence(case3);
//        }

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();


    }
}
