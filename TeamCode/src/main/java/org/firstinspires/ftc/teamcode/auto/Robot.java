package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.opencv.core.Mat;

import java.util.function.DoubleSupplier;
@Config
public class Robot {
    public Motor FL,FR,BL,BR;
    public MecanumDrive drive;
    public static PIDFCoefficients drivePIDF = new PIDFCoefficients(0.001,0,0,0);
    public static PIDFCoefficients headingPIDF = new PIDFCoefficients(-0.008,0,0,0);
    public static double TICKS_PER_REV = 145;
    public static double WHEEL_RADIUS = 9.6; // cm
    public static double GEAR_RATIO = 0.5;
    public PIDFController driveController;
    public PIDFController headingController;
    public DoubleSupplier heading;
    public LinearOpMode context;

    public Robot(Motor FL,
                 Motor FR,
                 Motor BL,
                 Motor BR, DoubleSupplier heading, LinearOpMode context){
        this.FL = FL;
        this.FR = FR;
        this.BL = BL;
        this.BR = BR;
        FL.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.context = context;
        this.heading = heading;
        drive = new MecanumDrive(FL,FR,BL,BR);
        driveController = new PIDFController(drivePIDF.p,
                drivePIDF.i, drivePIDF.d, drivePIDF.f);
        headingController = new PIDFController(headingPIDF.p,
                headingPIDF.i, headingPIDF.d, headingPIDF.f);
    }

    public double getAverageTicks(){
        return (double) (FL.motor.getCurrentPosition() + FR.motor.getCurrentPosition()
                + BL.motor.getCurrentPosition() + BR.motor.getCurrentPosition()) /4;
    }

    public void resetEncoders(){
        FL.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void runEncoders(){
        FL.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public void forward(double cm, double multiplier){
//        resetEncoders();
//        runEncoders();
//        while(!context.isStopRequested() && context.opModeIsActive()){
//            double averageTicks = getAverageTicks();
//            double result = Range.clip(driveController.calculate(averageTicks, cmToTicks(cm)),-1,1)*multiplier;
//            if(Math.abs(result) < 0.1){
//                break;
//            }
//            context.telemetry.addData("result", result);
//            context.telemetry.update();
//            powerAll(result,result,result,result);
//        }
//        powerAll(0,0,0,0);
        powerAll(0.3*multiplier,0.3*multiplier,0.3*multiplier,0.3*multiplier);
        try {
            Thread.sleep((long) cm);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        powerAll(0,0,0,0);

    }
    public void strafe(double cm, double multiplier){
//        resetEncoders();
//        runEncoders();
//        while(!context.isStopRequested() && context.opModeIsActive()){
//            double averageTicks = getAverageTicks();
//            double result = Range.clip(driveController.calculate(averageTicks, cmToTicks(cm)),-1,1)*multiplier;
//            if(Math.abs(result) < 0.1){
//                break;
//            }
//            context.telemetry.addData("result", result);
//            context.telemetry.update();
//            powerAll(result,result,result,result);
//        }
//        powerAll(0,0,0,0);
        powerAll(0.3*multiplier,-0.3*multiplier,-0.3*multiplier,0.3*multiplier);
        try {
            Thread.sleep((long) cm);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        powerAll(0,0,0,0);

    }
    public void powerAll(double FL, double FR, double BL, double BR){
        this.FL.set(-FL);
        this.FR.set(-FR);
        this.BL.set(-BL);
        this.BR.set(-BR);
    }

    public void turn(double target, double multiplier){
        while(!context.isStopRequested() && context.opModeIsActive()) {
            double result = Range.clip(headingController.calculate(heading.getAsDouble(), target),-1,1)*multiplier;
            context.telemetry.update();
            if(Math.abs(result) < 0.1){
                break;
            }
            powerAll(-result,result,-result,result);
        }
    }

    public int cmToTicks(double cm){
        return (int) (cm/(WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO / TICKS_PER_REV));
    }

}
