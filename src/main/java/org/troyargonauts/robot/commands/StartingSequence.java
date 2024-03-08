package org.troyargonauts.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.troyargonauts.robot.Robot;
import org.troyargonauts.robot.subsystems.Shooter;

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
                new InstantCommand(() -> Robot.getArm().setPower(0.08), Robot.getArm()),
                new WaitCommand(0.3),
                new InstantCommand(() -> Robot.getShooter().setState(Shooter.ShooterStates.RAMPUP), Robot.getShooter()),
                new InstantCommand(() -> Robot.getArm().setPower(-0.08), Robot.getArm()).until(() -> Robot.getArm().getLimitSwitch()),
                new InstantCommand(() -> Robot.getArm().setDesiredTarget(0), Robot.getArm())
        );
    }
}