package org.troyargonauts.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.troyargonauts.robot.Robot;
import org.troyargonauts.robot.commands.ShootingSequence;
import org.troyargonauts.robot.subsystems.Arm;
import org.troyargonauts.robot.subsystems.Shooter;

public class SH2Shoot extends SequentialCommandGroup {
    public SH2Shoot(){
        super(
                new InstantCommand(() -> Robot.getShooter().setState(Shooter.ShooterStates.WING_LINE), Robot.getShooter()),
                new InstantCommand(() -> Robot.getArm().setState(Arm.ArmStates.WING_LINE)),
                new WaitUntilCommand(Robot.getShooter()::isTopPidFinished),
                new ShootingSequence()
        );
    }
}
