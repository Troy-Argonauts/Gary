// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.troyargonauts.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.troyargonauts.common.math.OMath;
import org.troyargonauts.common.streams.IStream;
import org.troyargonauts.robot.commands.ShootInPlaceAuton;
import org.troyargonauts.robot.commands.ShootingSequence;
import org.troyargonauts.robot.commands.StartingSequence;
import org.troyargonauts.robot.generated.TunerConstants;

import org.troyargonauts.robot.subsystems.Arm.ArmStates;
import org.troyargonauts.robot.subsystems.Intake.IntakeStates;
import org.troyargonauts.robot.subsystems.Shooter;
import org.troyargonauts.robot.subsystems.Shooter.ShooterStates;

import java.util.function.BooleanSupplier;

import static org.troyargonauts.robot.Constants.Controllers.*;

/**
 * Class for setting up commands for the entire robot
 */
public class RobotContainer {


    /* Setting up bindings for necessary control of the swerve drive platform */

    public final CommandXboxController driver = new CommandXboxController(DRIVER);
    public final CommandXboxController operator = new CommandXboxController(OPERATOR);
    private final InstantCommand intakeIn = new InstantCommand(() -> Robot.getIntake().setState(IntakeStates.IN));
    private final InstantCommand intakeOff = new InstantCommand(() -> Robot.getIntake().setState(IntakeStates.OFF));

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

//    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
//        .withDeadband(Constants.Drivetrain.MAX_SPEED * 0.1).withRotationalDeadband(Constants.Drivetrain.MAX_ANGULAR_RATE * 0.1) // Add a 10% deadband
//        .withDriveRequestType(DriveRequestType.Velocity);

    public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.Drivetrain.MAX_SPEED * 0.08).withRotationalDeadband(Constants.Drivetrain.MAX_ANGULAR_RATE * 0.08) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.Velocity); // I want field-centric
                                                                // driving in open loop
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();


    /**
     * Configures controller button bindings for Teleoperated mode
     */
    public void configureBindings() {

        // driver controller commands
        drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
            drivetrain.applyRequest(
                () -> drive.withVelocityX(OMath.square(-driver.getLeftY()) * Constants.Drivetrain.MAX_SPEED) // Drive forward with
                // negative Y (forward)
                .withVelocityY(OMath.square(-driver.getLeftX()) * Constants.Drivetrain.MAX_SPEED) // Drive left with negative X (left)
                .withRotationalRate((-driver.getRightX()) * Constants.Drivetrain.MAX_ANGULAR_RATE) // Drive counterclockwise with negative X (left)
            )
        );


        driver.rightBumper().whileTrue(
            drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX())))
        );

        driver.povDown().onTrue(
            new InstantCommand(Robot.getClimber()::setTarget)
        );

        driver.b().onTrue(
                new InstantCommand(() -> System.out.println("Here"), Robot.getIntake())
        );

        Robot.getClimber().setDefaultCommand(
                new RunCommand(
                        () -> {
                            double climberSpeed = IStream.create(driver::getLeftTriggerAxis)
                                    .filtered(x -> OMath.deadband(x, DEADBAND))
                                    .get();
                            Robot.getClimber().setRawPower(climberSpeed);
                        }, Robot.getClimber()
                )
        );

        // operator controller commands
        Robot.getArm().setDefaultCommand(
            new RunCommand(
                () -> {
                    double armSpeed = IStream.create(operator::getRightY)
                        .filtered(x -> OMath.deadband(x, DEADBAND))
                        .get();
                    Robot.getArm().adjustSetpoint(-armSpeed);
                }, Robot.getArm()
            )
        );

        operator.a().onTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> Robot.getArm().setState(ArmStates.FLOOR_INTAKE), Robot.getArm())
//                new InstantCommand(() -> Robot.getIntake().setState(IntakeStates.IN), Robot.getIntake()).until(() -> Robot.getIntake().isNoteReady())
//                .andThen(new InstantCommand(() -> Robot.getIntake().setState(IntakeStates.OFF), Robot.getIntake()))
            )
        );

        operator.x().onTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> Robot.getShooter().setState(ShooterStates.AMP), Robot.getShooter()),
                new InstantCommand(() -> Robot.getArm().setState(ArmStates.AMP), Robot.getArm())
            )
        );

        operator.povDown().onTrue(
                new InstantCommand(() -> Robot.getShooter().setState(ShooterStates.OFF), Robot.getShooter())
        );

        operator.y().onTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> Robot.getShooter().setState(ShooterStates.STAGE), Robot.getShooter()),
                new InstantCommand(() -> Robot.getArm().setState(ArmStates.STAGE), Robot.getArm())
            )
        );

        operator.b().onTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> Robot.getShooter().setState(ShooterStates.SUBWOOFER), Robot.getShooter()),
                new InstantCommand(() -> Robot.getArm().setState(ArmStates.SUBWOOFER), Robot.getArm())
            )
        );

        operator.rightBumper().whileTrue(
            new ParallelCommandGroup(
                    new InstantCommand(() -> Robot.getShooter().setState(ShooterStates.THROWOUT), Robot.getShooter()),
                    new InstantCommand(() -> Robot.getIntake().setState(IntakeStates.OUT), Robot.getIntake())
            )
        ).whileFalse(
                new InstantCommand(() -> Robot.getIntake().setState(IntakeStates.OFF), Robot.getIntake())
        );

        operator.leftBumper().whileTrue(
            new ConditionalCommand(intakeIn,intakeOff, () -> !Robot.getIntake().isNoteReady())
        ).whileFalse(
                new InstantCommand(() -> Robot.getIntake().setState(IntakeStates.OFF), Robot.getIntake())
        );

        operator.rightTrigger().whileTrue(
           // new ShootingSequence().onlyIf(() -> operator.getRightTriggerAxis() > DEADBAND)
            new InstantCommand(() -> Robot.getIntake().setState(IntakeStates.IN), Robot.getIntake())
        ).whileFalse(
            new InstantCommand(() ->  Robot.getIntake().setState(IntakeStates.OFF), Robot.getIntake())
        );

        operator.povDown().onTrue(
            new InstantCommand(() -> Robot.getShooter().setState(ShooterStates.OFF), Robot.getShooter())
        );






    }

    /**
     * Runs configureBindings() method and registers Pathplanner NamedCommands
     */
    public RobotContainer() {
        NamedCommands.registerCommand("Starting Sequence", new StartingSequence());

        NamedCommands.registerCommand("Shooting Sequence Subwoofer",
            new InstantCommand(() -> Robot.getShooter().setState(Shooter.ShooterStates.SUBWOOFER), Robot.getShooter())
                    .until(() -> Robot.getShooter().isTopPidFinished())
                    .andThen(new ShootingSequence())

                //            new InstantCommand(() -> Robot.getShooter().setDesiredTarget(100, 100), Robot.getShooter())
//                .until(() -> Robot.getShooter().isTopPidFinished() && Robot.getShooter().isBottomPidFinished())
//            .alongWith(new InstantCommand(() -> Robot.getArm().setDesiredTarget(100), Robot.getArm()))
//                .until(() -> Robot.getArm().isPIDFinished())
//            .andThen(new ShootingSequence())
//            .andThen(new InstantCommand(() -> Robot.getShooter().setDesiredTarget(10, 10), Robot.getShooter()))
        );

        NamedCommands.registerCommand("Shooting Sequence W2", 
            new InstantCommand(() -> Robot.getShooter().setDesiredTarget(100, 100), Robot.getShooter())
                .until(() -> Robot.getShooter().isTopPidFinished() && Robot.getShooter().isBottomPidFinished())
            .alongWith(new InstantCommand(() -> Robot.getArm().setDesiredTarget(100), Robot.getArm()))
                .until(() -> Robot.getArm().isPIDFinished())
            .andThen(new ShootingSequence())
            .andThen(new InstantCommand(() -> Robot.getShooter().setDesiredTarget(10, 10), Robot.getShooter()))
        );

        NamedCommands.registerCommand("Shooting Sequence W3",
            new InstantCommand(() -> Robot.getShooter().setState(ShooterStates.STAGE), Robot.getShooter())
                .until(() -> Robot.getShooter().isTopPidFinished())
            .alongWith(new InstantCommand(() -> Robot.getArm().setState(ArmStates.STAGE), Robot.getArm()))
                .until(() -> Robot.getArm().isPIDFinished())
            .andThen(new ShootingSequence())
        );

        NamedCommands.registerCommand("Shooting Sequence SH1", 
            new InstantCommand(() -> Robot.getShooter().setDesiredTarget(100, 100), Robot.getShooter())
                .until(() -> Robot.getShooter().isTopPidFinished() && Robot.getShooter().isBottomPidFinished())
            .alongWith(new InstantCommand(() -> Robot.getArm().setDesiredTarget(100), Robot.getArm()))
                .until(() -> Robot.getArm().isPIDFinished())
            .andThen(new ShootingSequence())
            .andThen(new InstantCommand(() -> Robot.getShooter().setDesiredTarget(10, 10), Robot.getShooter()))
        );

        NamedCommands.registerCommand("Shooting Sequence SH2", 
            new InstantCommand(() -> Robot.getShooter().setDesiredTarget(100, 100), Robot.getShooter())
                .until(() -> Robot.getShooter().isTopPidFinished() && Robot.getShooter().isBottomPidFinished())
            .alongWith(new InstantCommand(() -> Robot.getArm().setDesiredTarget(100), Robot.getArm()))
                .until(() -> Robot.getArm().isPIDFinished())
            .andThen(new ShootingSequence())
            .andThen(new InstantCommand(() -> Robot.getShooter().setDesiredTarget(10, 10), Robot.getShooter()))
        );

        NamedCommands.registerCommand("Floor Intake", 
            new InstantCommand(() -> Robot.getArm().setState(ArmStates.FLOOR_INTAKE), Robot.getArm())
            .alongWith(new InstantCommand(() -> Robot.getIntake().setState(IntakeStates.IN), Robot.getIntake()).until(() -> Robot.getIntake().isNoteReady()))
            .andThen(new InstantCommand(() -> Robot.getIntake().setState(IntakeStates.OFF), Robot.getIntake()))
        );

        configureBindings();
    }
    /**
     * Gets boolean value of Operator controller RightBumper. Used to regulate intake when beam break is activated.
     */
    public boolean getOperatorRightBumper(){
        return operator.rightBumper().getAsBoolean();
    }
    /**
     * Gets boolean value of Operator controller RightTrigger. Used to regulate shooter when note is ready.
     */
    public boolean getOperatorRightTrigger(){
        return operator.rightTrigger().getAsBoolean();
    }
    public boolean getOperatorX(){
        return operator.x().getAsBoolean();
    }
    public CommandSwerveDrivetrain getDrivetrain(){
        return drivetrain;
    }

    public CommandXboxController getOperator(){
        return operator;
    }

    public CommandXboxController getDriver(){
        return driver;
    }
}



         
