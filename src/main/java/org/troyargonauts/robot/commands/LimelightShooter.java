package org.troyargonauts.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.troyargonauts.robot.Robot;

public class LimelightShooter extends SubsystemBase {
    public LimelightShooter(double distance){
        super(
                new InstantCommand(() -> Robot.getShooter().setDesiredTarget(Robot.getLimelight().calculateShooterSetpoint(distance)), Robot.getShooter())
        );
    }
}
