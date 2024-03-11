package org.troyargonauts.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.troyargonauts.robot.Robot;

public class LimelightArm extends SubsystemBase {
    public LimelightArm(double distance){
        super(
                new InstantCommand(() -> Robot.getArm().setDesiredTarget(Robot.getLimelight().calculateArmSetpoint(distance)), Robot.getArm())
        );
    }
}
