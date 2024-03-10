package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class DistanceSensorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Rev2mDistanceSensor distanceSensor = (Rev2mDistanceSensor)hardwareMap.get(DistanceSensor.class, "distSensor");
        waitForStart();
        while(opModeIsActive() && !isStopRequested()){
            telemetry.addData("dist", distanceSensor.getDistance(DistanceUnit.MM));
            telemetry.update();
        }
    }
}
