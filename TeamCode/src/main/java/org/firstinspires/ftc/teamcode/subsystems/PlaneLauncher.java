package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class PlaneLauncher {
    ServoEx servo;
    public static double minAngle = 0;
    public static double maxAngle = 360;
    public PlaneLauncher(HardwareMap hardwareMap){
        servo = new SimpleServo(hardwareMap, "planeLauncher",
                minAngle, maxAngle, AngleUnit.DEGREES);
        servo.setPosition(minAngle);
    }

    public void checkForLaunch(boolean launch){
        servo.setPosition(launch ? maxAngle : minAngle);
    }
}
