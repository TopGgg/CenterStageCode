package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.RunCommand;



public class LiftCommand extends RunCommand {
    public LiftCommand(LiftSubsystem liftSubsystem) {
        super(liftSubsystem::operate, liftSubsystem);
    }
}
