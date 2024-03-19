package org.troyargonauts.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import org.troyargonauts.robot.Robot;
import org.troyargonauts.robot.subsystems.Arm;
import org.troyargonauts.robot.subsystems.Intake;
import org.troyargonauts.robot.subsystems.Shooter;

public class FloorIntake extends SequentialCommandGroup {
    public FloorIntake(){
        super(
                new InstantCommand(() -> Robot.getArm().setState(Arm.ArmStates.FLOOR_INTAKE), Robot.getArm()),
//                new InstantCommand(() -> Robot.getShooter().setState(Shooter.ShooterStates.OFF), Robot.getShooter()),
                new WaitUntilCommand(() -> Robot.getArm().isPIDFinished()),
                new InstantCommand(() -> Robot.getIntake().setState(Intake.IntakeStates.IN), Robot.getIntake())
            );
    }
}
