package org.troyargonauts.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.troyargonauts.robot.Robot;
import org.troyargonauts.robot.subsystems.Intake.IntakeStates;

/**
 * Class representing Command Group for robot Shooting Sequence
 */
public class ShootingSequence extends SequentialCommandGroup {
    /**
     * Sets the IntakeState to IN for a specified amount of time, then sets the IntakeState to OFF
     */
    public ShootingSequence() {
        super(
            new InstantCommand(() -> Robot.getIntake().setState(IntakeStates.IN), Robot.getIntake()),
            new WaitCommand(3),
            new InstantCommand(() -> Robot.getIntake().setState(IntakeStates.OFF), Robot.getIntake())
        );
    }
}