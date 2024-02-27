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

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Auto", group="Auto", preselectTeleOp = "Main Tele Op")
@Config
public class Autonomous extends LinearOpMode{

    enum SIDE{
        RIGHT,
        LEFT
    }

    public static int element_zone = 3;

    private PropDetection propDetection = null;
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
        propDetection = new PropDetection(hardwareMap);

        telemetry.addData("Object Creation", "Done");
        telemetry.update();
    }



    public void runOpMode(){

        HardwareStart();

        String curAlliance = "red";
//        waitForStart();
        while (!opModeIsActive() && !isStopRequested()){
            element_zone = propDetection.elementDetection(telemetry);
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

        if(element_zone == 2){
            //2
            powerAll(0.3,0.3,0.3,0.3);
            sleep_(950);
            powerAll(0,0,0,0);
            intake.set(1);
            sleep_(1500);
            powerAll(-0.3,-0.3,-0.3,-0.3);
            sleep_(700);
            powerAll(0,0,0,0);
            intake.set(0);
        }else if(element_zone == 3){
            //3
            powerAll(0.3,0.3,0.3,0.3);
            sleep_(850);
            powerAll(-0.3,0.3,-0.3,0.3);
            sleep_(570);
            powerAll(0,0,0,0);
            sleep_(100);
            powerAll(0.3,0.3,0.3,0.3);
            sleep_(350);
            powerAll(0,0,0,0);
            intake.set(1);
            sleep_(1500);
            powerAll(-0.3,-0.3,-0.3,-0.3);
            sleep_(500);
            powerAll(0,0,0,0);
            intake.set(0);
        }else {
            //1
            powerAll(0.3,0.3,0.3,0.3);
            sleep_(850);
            powerAll(0.3,-0.3,0.3,-0.3);
            sleep_(570);
            powerAll(0,0,0,0);
            sleep_(100);
            powerAll(0.3,0.3,0.3,0.3);
            sleep_(350);
            powerAll(0,0,0,0);
            intake.set(1);
            sleep_(1500);
            powerAll(-0.3,-0.3,-0.3,-0.3);
            sleep_(500);
            powerAll(0,0,0,0);
            intake.set(0);

        }


    }
    public void sleep_(long ms){
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
public void powerAll(double FL, double FR, double BL, double BR){
        this.FL.set(FL);
        this.FR.set(-FR);
        this.BL.set(BL);
        this.BR.set(-BR);
}

}