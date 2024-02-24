package org.troyargonauts.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.troyargonauts.robot.Robot;

/**
 * Class representing the command group for the robot's Starting Sequence to ensure Arm encoder values are reset at the start of match
 */
public class StartingSequence extends SequentialCommandGroup {
    /**
     * Sets the Arm to a raw power until the Arm limit switch is active, then sets the Arm target position to 0.
     * Corresponds with if statement in robotInit() that does not start Arm PID until limit switch has been pressed once.
     */
    public StartingSequence() {
        super(
            new InstantCommand(() -> Robot.getArm().setPower(0.1), Robot.getArm()).until(() -> Robot.getArm().getLimitSwitch()),
            new InstantCommand(() -> Robot.getArm().setDesiredTarget(0), Robot.getArm())
        );
    }
}