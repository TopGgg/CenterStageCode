package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Driving.MainTeleOp.heading;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.mathUtils.AngleUtils;

import java.util.function.BooleanSupplier;

@Config
public class AutoGrip extends SubsystemBase {
    public static class AutoGripCommand extends RunCommand {
        public AutoGripCommand(AutoGrip subsystem) {
            super(subsystem::detect, subsystem);
        }
    }
    public Rev2mDistanceSensor dist;
    public LiftSubsystem lift;
    public BooleanSupplier toggle;
    public Servo locker;
    public static int threshold = 90;
    public static boolean active = true;
    public static boolean isPixelPresent = false;
    public AutoGrip(DistanceSensor dist,Servo locker, LiftSubsystem lift, BooleanSupplier toggle){
        this.dist = (Rev2mDistanceSensor) dist;
        this.lift = lift;
        this.locker = locker;
        this.toggle = toggle;
    }

    public void detect(){
        if(toggle.getAsBoolean()){
            active = !active;
        }
        if(active){
            isPixelPresent = dist.getDistance(DistanceUnit.MM) <= threshold;
            if(isPixelPresent){
                locker.setPosition(0);
//                lift.setGripperState(true);
            }else{
//                lift.setGripperState(false);
                locker.setPosition(1);
            }
        }
    }

}
