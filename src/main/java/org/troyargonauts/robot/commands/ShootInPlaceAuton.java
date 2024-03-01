package org.troyargonauts.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.troyargonauts.robot.Robot;
import org.troyargonauts.robot.subsystems.Shooter;

public class ShootInPlaceAuton extends SequentialCommandGroup {
    public ShootInPlaceAuton(){
        super(
                new StartingSequence(),
                new InstantCommand(() -> Robot.getShooter().setState(Shooter.ShooterStates.SUBWOOFER), Robot.getShooter()),
                new WaitUntilCommand(Robot.getArm()::getLimitSwitch),
                new ShootingSequence()
            );
    }
}
