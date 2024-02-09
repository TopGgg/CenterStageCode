package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.RunCommand;
import org.firstinspires.ftc.teamcode.SimpleTeleSample.subsystems.DriveSubsystem;


public class DriveCommand extends RunCommand {
    public DriveCommand(DriveSubsystem driveSubsystem) {
        super(driveSubsystem::drive, driveSubsystem);
    }
}
