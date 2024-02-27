package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.RunCommand;


public class IntakeCommand extends RunCommand {
    public IntakeCommand(IntakeSubsystem intakeSubsystem) {
        super(intakeSubsystem::operate, intakeSubsystem);
    }
}
