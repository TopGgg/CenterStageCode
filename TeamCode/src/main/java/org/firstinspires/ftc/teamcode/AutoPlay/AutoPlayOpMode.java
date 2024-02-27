package org.firstinspires.ftc.teamcode.AutoPlay;

import static org.firstinspires.ftc.teamcode.AutoPlay.Board.score;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Arrays;

@TeleOp
public class AutoPlayOpMode extends LinearOpMode {

    Board.Backdrop game;
    @Override
    public void runOpMode() throws InterruptedException {
//        BoardDetection detection = new BoardDetection(hardwareMap);
//        detection.init();
//        detection.start();
//        game = new Board.Backdrop();
//        game.findPixel(1,0).type = Board.PIXEL_TYPE.GREEN;
//        game.findPixel(2,0).type = Board.PIXEL_TYPE.PURPLE;
//        game.findPixel(2,1).type = Board.PIXEL_TYPE.YELLOW;
//
//        game.findPixel(4,0).type = Board.PIXEL_TYPE.GREEN;
//        game.findPixel(5,0).type = Board.PIXEL_TYPE.GREEN;
//        game.findPixel(5,1).type = Board.PIXEL_TYPE.GREEN;

        waitForStart();
        float[] hsv = new float[3];
        Color.RGBToHSV(255,0,0, hsv);
        telemetry.addData("H", hsv[0]);
        telemetry.addData("S", hsv[1]);
        telemetry.addData("V", hsv[2]);
        telemetry.update();
//
//        findBestActions();
//        while (!gamepad1.a);
//        Thread.sleep(1900);
//        findBestActions();
        while (!isStopRequested() && opModeIsActive());
    }

    public void findBestActions(){
        long start = System.currentTimeMillis();
        Board.Action[] actions = game.getBestActions();


        if(actions != null){
            telemetry.addData("Actions: ", Arrays.toString(actions));
            game.findPixel(actions[0].x, actions[0].y).type = actions[0].type;
            game.findPixel(actions[1].x, actions[1].y).type = actions[1].type;
            game.findPixel(actions[2].x, actions[2].y).type = actions[2].type;
        }


//        telemetry.addData("time",System.currentTimeMillis() - start);
        telemetry.addData("Score", score);
        telemetry.update();
    }
}
