package org.firstinspires.ftc.teamcode.AutoIntake;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class AutoIntake extends SubsystemBase {

    public SampleMecanumDrive drive;
    public Motor intake;

    public static class AutoIntakeCommand extends RunCommand {
        public AutoIntakeCommand(AutoIntake subsystem){
            super(subsystem::check, subsystem);
        }
    }

    public AutoIntake(Motor intake, SampleMecanumDrive drive){
        this.drive = drive;
        this.intake = intake;
    }

    public void check(){
        Pose2d pose = drive.getPoseEstimate();

    }
}
