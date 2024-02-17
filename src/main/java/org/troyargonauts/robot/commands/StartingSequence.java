package org.troyargonauts.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.troyargonauts.robot.Robot;

public class StartingSequence extends SequentialCommandGroup {
    public StartingSequence() {
        super(
            new InstantCommand(() -> Robot.getArm().setPower(0.1), Robot.getArm()).until(() -> Robot.getArm().getLimitSwitch()),
            new InstantCommand(() -> Robot.getArm().setDesiredTarget(0), Robot.getArm())
        );
    }
}