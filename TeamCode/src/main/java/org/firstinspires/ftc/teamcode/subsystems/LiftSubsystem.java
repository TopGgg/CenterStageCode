package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Driving.MainTeleOp.heading;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

@Config
public class LiftSubsystem extends SubsystemBase {

    public enum StickState{
        BOARD,
        GRIP,
        NORMAL
    }
    private DoubleSupplier liftSpeed;
    public Motor lift;
    private Telemetry telemetry;
    public static double speedMultiplier = 0.65;
    private Servo stick;
    private Servo rotation;
    private Servo gripper;

    public static double boardPositionStick = 0.73;
    public static double gripPositionStick = 0.24;
    public static double normalPositionStick = 0.35;
    public static double boardPositionRotation = 0;
    public static double gripPositionRotation = 0.88;
    public static double gripPositionGripper = 1;
    public static double ungripPositionGripper = 0;
    public static double ticksPerLevel = 100;

    public BooleanSupplier stickBtn;
    public BooleanSupplier normalBtn;
    public BooleanSupplier rotBtn;
    public BooleanSupplier gripBtn;
    public BooleanSupplier liftUpBtn;
    public BooleanSupplier lift1Btn;
    public BooleanSupplier lift2Btn;
    public BooleanSupplier lift3Btn;
    public BooleanSupplier lift4Btn;
    public BooleanSupplier resetBtn;
    public Runnable vibrate;

    public StickState stickState;

    public Runnable readButtons;

    public boolean isRawControl = true;
    public PIDFCoefficients pidf = new PIDFCoefficients(0.01,0,0,0);

    public int level = -1;
    public int integral;
    public int integral_prior;
    public int error_prior;
    public boolean gripState;

    public Rev2mDistanceSensor gripSensor;


    public LiftSubsystem(Motor lift,
                         Servo stick,
                         Servo rotation,
                         Servo gripper,
                         DistanceSensor gripSensor,
                         GamepadEx gamepad,
                         Telemetry telemetry) {
        this.gripSensor = (Rev2mDistanceSensor) gripSensor;
        if(gamepad != null){
            liftSpeed = ()->-gamepad.getLeftY();

            stickBtn = () -> gamepad.wasJustReleased(GamepadKeys.Button.B);
            rotBtn = () -> gamepad.wasJustReleased(GamepadKeys.Button.X);
            gripBtn = () -> gamepad.wasJustReleased(GamepadKeys.Button.A);
            normalBtn = () -> gamepad.wasJustReleased(GamepadKeys.Button.DPAD_DOWN);
//            liftUpBtn = () -> gamepad.getButton(GamepadKeys.Button.LEFT_BUMPER);
//            resetBtn = () -> gamepad.getButton(GamepadKeys.Button.BACK);
//
//            lift1Btn = () -> gamepad.wasJustReleased(GamepadKeys.Button.DPAD_DOWN);
//            lift2Btn = () -> gamepad.wasJustReleased(GamepadKeys.Button.DPAD_RIGHT);
//            lift3Btn = () -> gamepad.wasJustReleased(GamepadKeys.Button.DPAD_LEFT);
//            lift4Btn = () -> gamepad.wasJustReleased(GamepadKeys.Button.DPAD_UP);

            readButtons = gamepad::readButtons;
        }
        this.lift = lift;
        this.telemetry = telemetry;
        this.stick = stick;
        this.rotation = rotation;
        this.gripper = gripper;
        lift.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stick.setDirection(Servo.Direction.REVERSE);
        setStickState(StickState.NORMAL);
        setRotationState(StickState.GRIP);
        setGripperState(true);
    }




    public void operate() {
//        readButtons.run();
//        if(lift1Btn.getAsBoolean()){
//            level++;
//        }
//        if(lift2Btn.getAsBoolean()){
//            level+=2;
//        }
//        if(lift3Btn.getAsBoolean()){
//            level+=3;
//        }
//        if(lift4Btn.getAsBoolean()){
//            level+=4;
//        }
//        if(resetBtn.getAsBoolean()){
//            level = 0;
//        }

        lift.set(liftSpeed.getAsDouble()*0.6);
//        if(liftSpeed.getAsDouble() != 0){
//            if(!isRawControl){
//                isRawControl = true;
//                level = -1;
//            }
//
//        }
//        else if(level>=0){
//            if(isRawControl){
//                lift.stopMotor();
//                isRawControl = false;
//            }
//            int error = (int) ((level*ticksPerLevel) - lift.getCurrentPosition());
//            integral = integral_prior + error;
//            double correction = Range.clip(pidf.p*error+(error-error_prior)*pidf.d+
//                    integral_prior*pidf.i+pidf.f,-0.5,0.5);
//            telemetry.addData("correction", correction);
//            lift.set(correction);
//            error_prior = error;
//            integral_prior = integral;
//
//        }

        if(stickBtn.getAsBoolean()){
            if(stickState == StickState.BOARD){
                stickState = StickState.GRIP;
            }else{
                stickState = StickState.BOARD;
            }
            setStickState(stickState);
            setRotationState(stickState);

        }
        if(normalBtn.getAsBoolean()){
            if(stickState == StickState.BOARD || stickState == StickState.GRIP){
                stickState = StickState.NORMAL;
            }else if(stickState == StickState.NORMAL){
                stickState = StickState.GRIP;
            }
            setStickState(stickState);
        }
        if(gripBtn.getAsBoolean()){
            gripState = !gripState;
            if(gripState){

            }
        }

        setGripperState(gripSensor.getDistance(DistanceUnit.MM) < 90);


    }

    public void goTo(int level){
        double ticks = level * ticksPerLevel;

    }

    public void setStickState(StickState state){
        if (state == StickState.BOARD){
            stick.setPosition(boardPositionStick);
        }else if (state == StickState.GRIP){
            stick.setPosition(gripPositionStick);
        }else if(state == StickState.NORMAL){
            stick.setPosition(normalPositionStick);
        }
    }

    public void setRotationState(StickState state){
        if (state == StickState.BOARD){
            rotation.setPosition(boardPositionRotation);
        }else if (state == StickState.GRIP){
            rotation.setPosition(gripPositionRotation);
        }
    }

    public void setGripperState(boolean grip){
        if (grip){
            gripper.setPosition(gripPositionGripper);
        }else{
            gripper.setPosition(ungripPositionGripper);
        }
    }

}