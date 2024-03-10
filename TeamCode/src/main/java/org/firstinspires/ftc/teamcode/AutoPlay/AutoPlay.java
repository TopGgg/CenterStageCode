package org.firstinspires.ftc.teamcode.AutoPlay;

import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.vision.BackdropDetectionPipeline;
import org.opencv.core.Mat;

import java.util.function.BooleanSupplier;

public class AutoPlay extends SubsystemBase {
    BoardDetection detection;
    BooleanSupplier toggle;
    public static int score = -1;
    public static String action1 = "";
    public static String action2 = "";
    public static String action3 = "";

    public static class AutoPlayCommand extends RunCommand {
        public AutoPlayCommand(AutoPlay autoPlay){
            super(autoPlay::plan,autoPlay);
        }
    }

    public AutoPlay(LinearOpMode context, HardwareMap hardwareMap, Telemetry telemetry, GamepadEx operator){
        detection = new BoardDetection(hardwareMap, telemetry);
        detection.init();
        new Thread(()->{
            while (context.opModeIsActive() && !context.isStopRequested()){
                if(BoardDetectionPipeline.active) {
                    score = detection.board.score();
                    Board.Action[] actions = detection.board.getBestActions();
                    action1 = actions[0].toString();
                    action2 = actions[1].toString();
                    action3 = actions[2].toString();
                }
                double start = System.currentTimeMillis();
                while ((System.currentTimeMillis() - start) < 3000 && context.opModeIsActive() && !context.isStopRequested());
            }
        }).start();
        toggle = () -> operator.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT);
    }

    public void plan(){
        if(toggle.getAsBoolean()){
            BackdropDetectionPipeline.active = !BackdropDetectionPipeline.active;
        }
    }
}
