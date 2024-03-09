package org.troyargonauts.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.troyargonauts.robot.Robot;
import org.troyargonauts.robot.RobotContainer;
import org.troyargonauts.robot.subsystems.Arm;
import org.troyargonauts.robot.subsystems.Intake;
import org.troyargonauts.robot.subsystems.Shooter;

public class Rumble extends SequentialCommandGroup {
    /**
     * Sets the IntakeState to IN for a specified amount of time, then sets the IntakeState to OFF
     */
    public Rumble() {
        super(
                new InstantCommand(() -> Robot.getRobotContainer().getOperator().getHID().setRumble(GenericHID.RumbleType.kBothRumble,0.5), Robot.getIntake()),
                new InstantCommand(() -> Robot.getRobotContainer().getDriver().getHID().setRumble(GenericHID.RumbleType.kBothRumble,0.5), Robot.getIntake()),
                new WaitCommand(1),
                new InstantCommand(() -> Robot.getRobotContainer().getOperator().getHID().setRumble(GenericHID.RumbleType.kBothRumble,0), Robot.getIntake()),
                new InstantCommand(() -> Robot.getRobotContainer().getDriver().getHID().setRumble(GenericHID.RumbleType.kBothRumble,0), Robot.getIntake())
                );

    }
}