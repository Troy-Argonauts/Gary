package org.troyargonauts.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.troyargonauts.common.math.OMath;
import org.troyargonauts.robot.CommandSwerveDrivetrain;
import org.troyargonauts.robot.Constants;
import org.troyargonauts.robot.Robot;
import org.troyargonauts.robot.RobotContainer;

public class TuneDrive extends SequentialCommandGroup {

    public TuneDrive(){
        super(
        new RunCommand(() -> Robot.getRobotContainer()
                .getDrivetrain()
                .applyRequest(() -> Robot.getRobotContainer().drive
                        .withVelocityX(0.25 * Constants.Drivetrain.MAX_SPEED)
                        .withVelocityY(0.25 * Constants.Drivetrain.MAX_SPEED)
                        .withRotationalRate(0)), Robot.getRobotContainer().getDrivetrain()).withTimeout(5)
        );
    }
}
