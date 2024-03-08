package org.troyargonauts.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.troyargonauts.robot.Robot;
import org.troyargonauts.robot.subsystems.Arm;
import org.troyargonauts.robot.subsystems.Shooter;

public class W2Shoot extends SequentialCommandGroup {
    public W2Shoot(){
        super(
                new InstantCommand(() -> Robot.getShooter().setState(Shooter.ShooterStates.WING_NOTE), Robot.getShooter()),
                new InstantCommand(() -> Robot.getArm().setState(Arm.ArmStates.WING_NOTE)),
                new WaitUntilCommand(Robot.getArm()::isPIDFinished),
                new WaitUntilCommand(Robot.getShooter()::isTopPidFinished),
                new ShootingSequence()
        );
    }
}
