package org.troyargonauts.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.troyargonauts.robot.Robot;
import org.troyargonauts.robot.subsystems.Arm;
import org.troyargonauts.robot.subsystems.Intake.IntakeStates;
import org.troyargonauts.robot.subsystems.Shooter;

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
            new WaitCommand(0.5),
            new InstantCommand(() -> Robot.getIntake().setState(IntakeStates.OFF), Robot.getIntake()),
            new InstantCommand(() -> Robot.getArm().setState(Arm.ArmStates.FLOOR_INTAKE), Robot.getArm()),
            new InstantCommand(() -> Robot.getShooter().setState(Shooter.ShooterStates.SUBWOOFER), Robot.getShooter())
        );
    }
}