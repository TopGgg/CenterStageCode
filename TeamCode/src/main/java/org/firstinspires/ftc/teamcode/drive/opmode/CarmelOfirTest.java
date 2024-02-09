package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.distanceMultiplier;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.Set;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class CarmelOfirTest extends LinearOpMode {
//starting -36 64.2
    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        byte w = 1;

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(90));

        drive.setPoseEstimate(startPose);


        //Trajectory trajectory1 = drive.trajectoryBuilder(startPose)
        //    .forward(-27.5)
        //    .build();

        //Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
        //        .forward(27.5)
        //        .build();

        Trajectory trajectory4 = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(-36,64.2), Math.toRadians(180))

                .addDisplacementMarker(()->{

                })
                .splineTo(new Vector2d(0,0), Math.toRadians(180))
                .build();

        Trajectory trajectory3 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(27.5, 0), Math.toRadians(0))
                .build();


        waitForStart();

        if (isStopRequested()) return;

        if (w==1) {
            drive.followTrajectory(trajectory4);
            //motor stuff
//            drive.followTrajectory(trajectory2);
        } else if (w==2) {
            drive.followTrajectory(trajectory3);
        };


        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }
}
