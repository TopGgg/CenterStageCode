package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class IMUUtils {

    IMU.Parameters params;
    IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle = 0;
    public static double fallAngle = 0;


    public IMUUtils(IMU imu){
        this.imu = imu;
//        params = new IMU.Parameters();
//        params.mode                = IMU.SensorMode.IMU;
//        params.angleUnit           = IMU.AngleUnit.DEGREES;
//        params.accelUnit           = IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        params.loggingEnabled      = false;
    }
    public void init(HardwareMap hardwareMap, LinearOpMode context){
//        imu = hardwareMap.get(IMU.class, "imu");
//        imu.initialize(params);
//        while (context.opModeIsActive() && !context.isStopRequested() && !imu.isGyroCalibrated())
//        {
//            try {
//                sleep(50);
//            } catch (InterruptedException e) {
//                throw new RuntimeException(e);
//            }
//            context.idle();
//        }
    }

    public double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;


        globalAngle += deltaAngle;


        lastAngles = angles;

        fallAngle = AngleUnit.DEGREES.normalize(angles.thirdAngle);

        return globalAngle;
    }

}
