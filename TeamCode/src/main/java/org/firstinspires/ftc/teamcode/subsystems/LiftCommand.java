package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.RunCommand;

import org.firstinspires.ftc.teamcode.SimpleTeleSample.subsystems.DriveSubsystem;


public class LiftCommand extends RunCommand {
    public LiftCommand(LiftSubsystem liftSubsystem) {
        super(liftSubsystem::operate, liftSubsystem);
    }
}
