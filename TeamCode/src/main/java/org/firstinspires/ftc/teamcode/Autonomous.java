package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import static org.firstinspires.ftc.teamcode.mathUtils.LengthUtils.cmToInch;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.PropDetection;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Auto", group="Auto")
@Config
public class Autonomous extends LinearOpMode{

    enum SIDE{
        RIGHT,
        LEFT
    }

    public static int element_zone = 2;

//    private PropDetection propDetection = null;
    Motor FR;
    Motor FL;
    Motor BR;
    Motor BL;
    Motor intake;

    public static final double intakeSpeed = 0.7;

    boolean togglePreview = true;
    public SIDE side;

    public void HardwareStart() {
        telemetry.addData("Object Creation", "Start");
        telemetry.update();

        FR = new Motor(hardwareMap, "FR");
        FL = new Motor(hardwareMap, "FL");
        BR = new Motor(hardwareMap, "BR");
        BL = new Motor(hardwareMap, "BL");
        intake = new Motor(hardwareMap, "intake");
//        propDetection = new PropDetection(hardwareMap);

        telemetry.addData("Object Creation", "Done");
        telemetry.update();
    }



    public void runOpMode(){

        HardwareStart();

        String curAlliance = "red";
//        waitForStart();
        while (!opModeIsActive() && !isStopRequested()){
//            element_zone = propDetection.elementDetection(telemetry);
//            telemetry.addData("getMaxDistance", propDetection.getMaxDistance());

//            if (togglePreview && gamepad2.a){
//                togglePreview = false;
//                propDetection.toggleAverageZone();
//            }else if (!gamepad2.a){
//                togglePreview = true;
//            }

            if(gamepad1.a){
                side = SIDE.RIGHT;
            }else if(gamepad1.y){
                side = SIDE.LEFT;
            }


            if (gamepad1.x){
                curAlliance = "blue";
            }else if (gamepad1.b){
                curAlliance = "red";
            }
//            propDetection.setAlliance(curAlliance);
            telemetry.addData("Select Alliance (Gamepad1 X = Blue, Gamepad1 B = Red)", "");
            telemetry.addData("Select Side (Gamepad1 A = Right, Gamepad1 Y = Left)", "");
            telemetry.addData("Current Alliance Selected : ", curAlliance.toUpperCase());
            telemetry.addData("Current Side Selected : ", side);


            telemetry.update();
        }
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setExternalHeading(Math.toRadians(-90));
        drive.setPoseEstimate(new Pose2d(-36,62));
        TrajectorySequence trajectory = drive.trajectorySequenceBuilder(new Pose2d(-36, 62, Math.toRadians(-90)))
                .forward(20)
//                .lineTo(new Vector2d(-36, 20))
//                .addDisplacementMarker(()->{intake.set(1);})
//                .lineTo(new Vector2d(-36, 40))
                .build();
        int zone = element_zone;
        telemetry.addData("Object", "Passed waitForStart");
        telemetry.update();
        if(zone == 2){
            drive.followTrajectorySequence(trajectory);
            intake.set(0);
        }


    }

}