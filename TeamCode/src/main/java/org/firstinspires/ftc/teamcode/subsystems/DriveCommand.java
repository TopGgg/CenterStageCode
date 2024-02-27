package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.RunCommand;


public class DriveCommand extends RunCommand {
    public DriveCommand(DriveSubsystem driveSubsystem) {
        super(driveSubsystem::drive, driveSubsystem);
    }
}
