package org.troyargonauts.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import org.troyargonauts.robot.Robot;
import org.troyargonauts.robot.subsystems.Arm;
import org.troyargonauts.robot.subsystems.Intake;
import org.troyargonauts.robot.subsystems.Shooter;

public class WaitUntilNoteReady extends SequentialCommandGroup {
    public WaitUntilNoteReady(){
        super(
                new WaitUntilCommand(() -> Robot.getIntake().isNoteReady()),
                new InstantCommand(() -> Robot.getIntake().setState(Intake.IntakeStates.OFF))
        );
    }
}
