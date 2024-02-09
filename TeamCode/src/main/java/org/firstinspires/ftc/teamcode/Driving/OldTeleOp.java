package org.firstinspires.ftc.teamcode.Driving;

import static org.firstinspires.ftc.teamcode.Autonomous.intakeSpeed;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.IMUUtils;
import org.firstinspires.ftc.teamcode.subsystems.PlaneLauncher;
import org.firstinspires.ftc.teamcode.mathUtils.AngleUtils;

@TeleOp
public class OldTeleOp extends LinearOpMode {

    OldDrive drive;
    GamepadEx driver;
    GamepadEx operator;
    Motor FR;
    Motor FL;
    Motor BR;
    Motor BL;
    Motor intake;
    PlaneLauncher planeLauncher;

    IMUUtils imu = new IMUUtils();

    double heading;
    double normalHeading;
    double correction;

    double powerModifier = 0.65;

    @Override
    public void runOpMode() throws InterruptedException {
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        FR = new Motor(hardwareMap, "FR");
        FL = new Motor(hardwareMap, "FL");
        BR = new Motor(hardwareMap, "BR");
        BL = new Motor(hardwareMap, "BL");

//        FL.motor.setDirection(DcMotorSimple.Direction.REVERSE);
//        BL.motor.setDirection(DcMotorSimple.Direction.REVERSE);

       intake = new Motor(hardwareMap, "intake");
        drive = new OldDrive(FR,FL,BR,BL);
        planeLauncher = new PlaneLauncher(hardwareMap);


        imu.init(hardwareMap, this);

        resetEncoder(FL);
        resetEncoder(FR);
        resetEncoder(BL);
        resetEncoder(BR);
        waitForStart();
        while(opModeIsActive() && !isStopRequested()){
            heading = imu.getAngle();
            normalHeading = AngleUtils.convertToNormalAngle(heading);
            correction = AngleUtils.checkCorrection(operator.getButton(GamepadKeys.Button.A),
                    normalHeading);
            checkPowerModifier();
            drive.drive(-driver.getLeftX() * powerModifier,
                    -driver.getLeftY() * powerModifier,
                    Range.clip(-driver.getRightX() * powerModifier + correction, -1, 1), heading);
            intake();
            if(driver.getButton(GamepadKeys.Button.A)){
                imu.init(hardwareMap, this);
            }
            planeLauncher.checkForLaunch(operator.getButton(GamepadKeys.Button.B));
            telemetry.addData("angle", heading);
            telemetry.addData("normalAngle", normalHeading);

            telemetry.addData("FL", -FL.motor.getCurrentPosition());
            telemetry.addData("FR", FR.motor.getCurrentPosition());
            telemetry.addData("BL", -BL.motor.getCurrentPosition());
            telemetry.addData("BR", BR.motor.getCurrentPosition());
            telemetry.update();
        }
    }

    private void resetEncoder(Motor motor){
        motor.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        motor.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done
    }

    private void checkPowerModifier(){
        driver.readButtons();
        if(driver.getButton(GamepadKeys.Button.RIGHT_BUMPER)
                && driver.getButton(GamepadKeys.Button.LEFT_BUMPER)){
            powerModifier = 1;

        }else if(driver.getButton(GamepadKeys.Button.RIGHT_BUMPER)){
            powerModifier = 0.35;
        }else if (driver.getButton(GamepadKeys.Button.LEFT_BUMPER)){
            powerModifier = 0.8;
        }else {
            powerModifier = 0.65;
        }
    }

    private void intake(){
        operator.readButtons();
        double right = operator.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)*intakeSpeed;
        double left = operator.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)*intakeSpeed;

        if(right > 0){
            intake.set(-right);
        }else if(left > 0){
            intake.set(left);
        }else {
            intake.set(0);
        }
    }


}
