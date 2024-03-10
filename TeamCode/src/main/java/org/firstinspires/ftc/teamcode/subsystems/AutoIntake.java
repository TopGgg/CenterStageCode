package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Driving.MainTeleOp.heading;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Driving.MainTeleOp;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.mathUtils.AngleUtils;

import java.util.function.BooleanSupplier;

public class AutoIntake extends SubsystemBase {
    public static class AutoIntakeCommand extends RunCommand {
        public AutoIntakeCommand(AutoIntake subsystem) {
            super(subsystem::detect, subsystem);
        }
    }
    public SampleMecanumDrive odometry;
    public IntakeSubsystem intake;
    public BooleanSupplier toggle;
    public static double threshold = 40;
    public static boolean active = false;
    public AutoIntake(SampleMecanumDrive odometry, IntakeSubsystem intake, BooleanSupplier toggle){
        this.intake = intake;
        this.odometry = odometry;
        this.toggle = toggle;
    }

    public void detect(){
        if(toggle.getAsBoolean()){
            active = !active;
        }
        if(active) {
            Pose2d pose = odometry.getPoseEstimate();
            Pose2d target = new Pose2d(-60, 60);
            double distance = Math.sqrt(Math.pow(pose.getX() - target.getX(), 2) + Math.pow(pose.getY() - target.getY(), 2));
            if (distance <= threshold) {
                intake.activate();
            } else {
                intake.stop();
            }
        }
    }

}
