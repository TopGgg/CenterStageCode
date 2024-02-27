package org.firstinspires.ftc.teamcode.AutoPlay;

import static org.firstinspires.ftc.teamcode.Driving.MainTeleOp.heading;
import static org.firstinspires.ftc.teamcode.Driving.MainTeleOp.normalHeading;

import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.mathUtils.AngleUtils;

public class CameraAligner extends SubsystemBase {
    public static class CameraAlignerCommand extends RunCommand {
        public CameraAlignerCommand(CameraAligner subsystem) {
            super(subsystem::align, subsystem);
        }
    }
    public Servo servo;
    public CameraAligner(Servo servo){
        this.servo = servo;
    }

    public void align(){
        double h = Math.floor(AngleUtils.convertToNormalAngle(heading-90));
        if(h > 270){
            h = 270;
        }
        double pos = Range.scale(h, 0,270,0,1);
        servo.setPosition(pos);
    }
}
