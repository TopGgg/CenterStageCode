package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Driving.MainTeleOp;
import org.firstinspires.ftc.teamcode.IMUUtils;

import java.util.function.BooleanSupplier;

public class AntiSlip extends SubsystemBase {
    public static class AntiSlipCommand extends RunCommand {
        public AntiSlipCommand(AntiSlip subsystem) {
            super(subsystem::detect, subsystem);
        }
    }


    public void detect(){
        double fall = IMUUtils.fallAngle;
        MainTeleOp.isFallingBack = fall < 0;
        MainTeleOp.isFalling = MainTeleOp.isFallingBack ? fall < -5 : fall > 3.5;
    }

}
