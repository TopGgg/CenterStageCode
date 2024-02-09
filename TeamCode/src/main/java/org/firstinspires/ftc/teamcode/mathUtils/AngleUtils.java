package org.firstinspires.ftc.teamcode.mathUtils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.Range;

@Config
public class AngleUtils {

    public static double Kp = 0.01;

    public static double convertToNormalAngle(double angle){
//        angle = -angle;
        if(angle > 0){
            return angle%360;
        } else if (angle < 0) {
            return 360-(Math.abs(angle)%360);
        }

        return 0;
    }

    public static double checkCorrection(boolean button, double heading) {
        if(!button)
            return 0;

        double correction = 0;
        if(heading < 180){
            //rotate to 90
            double error = heading - 90;
            correction = error*Kp;
        }else if(heading >= 180){
            //rotate to 270
            double error = heading - 270;
            correction = error*Kp;
        }
        return Range.clip(correction, -1, 1);
    }
}
