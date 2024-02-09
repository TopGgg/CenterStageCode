package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.RunCommand;

import org.firstinspires.ftc.teamcode.SimpleTeleSample.subsystems.DriveSubsystem;


public class IntakeCommand extends RunCommand {
    public IntakeCommand(IntakeSubsystem intakeSubsystem) {
        super(intakeSubsystem::operate, intakeSubsystem);
    }
}
