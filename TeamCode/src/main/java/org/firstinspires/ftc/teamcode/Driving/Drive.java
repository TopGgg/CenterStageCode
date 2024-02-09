package org.firstinspires.ftc.teamcode.Driving;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;

public class Drive {
    public MecanumDrive drive;

    public Drive(Motor FR, Motor FL, Motor BR, Motor BL){
        drive = new MecanumDrive(FL, FR, BL, BR);
    }

    public void drive(double strafe, double forward, double rotation, double heading){
        drive.driveFieldCentric(strafe, forward, rotation, heading);
    }

}
