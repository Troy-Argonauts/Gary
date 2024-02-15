package org.troyargonauts.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.troyargonauts.robot.Robot;
import org.troyargonauts.robot.subsystems.Intake;

public class ShootingSequence extends SequentialCommandGroup {
    public ShootingSequence(){
        super(
                new InstantCommand(() -> System.out.println("Shoot")),
                new InstantCommand(() -> Robot.getIntake().setState(Intake.MotorState.IN)),
                new InstantCommand(() -> Robot.getIntake().setState(Intake.MotorState.OFF))
        );
    }

}
