// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.troyargonauts.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.troyargonauts.common.math.OMath;
import org.troyargonauts.common.streams.IStream;
import org.troyargonauts.robot.generated.TunerConstants;
import org.troyargonauts.robot.subsystems.Climber;
import org.troyargonauts.robot.subsystems.Intake;

import static org.troyargonauts.robot.Robot.*;
import org.troyargonauts.robot.subsystems.Arm;
import org.troyargonauts.robot.subsystems.Climber;
import org.troyargonauts.robot.subsystems.Intake;
import org.troyargonauts.robot.subsystems.Shooter;


public class RobotContainer {
  private double MaxSpeed = 6; // 6 meters per second desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */

  private final CommandXboxController driver = new CommandXboxController(0); // My joystick
  private final CommandXboxController operator = new CommandXboxController(1);

  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.Velocity); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
      drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
              drivetrain.applyRequest(() -> drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with
                      // negative Y (forward)
                      .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                      .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
              ));

      driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
      driver.b().whileTrue(drivetrain
              .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

      // reset the field-centric heading on left bumper press
      driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));


      if (Utils.isSimulation()) {
          drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
          drivetrain.registerTelemetry(logger::telemeterize);
      }

      driver.rightBumper().onTrue(
              drivetrain.applyRequest(() -> brake)
      );

      operator.a().onTrue(
              new ParallelCommandGroup(
                      new InstantCommand(() -> Robot.getArm().setState(Arm.ArmStates.FLOOR_INTAKE)),
                      new InstantCommand(() -> Robot.getIntake().setState(Intake.MotorState.IN)).until(() -> !Robot.getIntake().isNoteReady())
                              .andThen(new InstantCommand(() -> Robot.getIntake().setState(Intake.MotorState.OFF)))
              )
      );

      operator.x().onTrue(
              new ParallelCommandGroup(
                      new InstantCommand(() -> Robot.getShooter().setTopState(Shooter.topStates.AMP)),
                      new InstantCommand(() -> Robot.getShooter().setBottomState(Shooter.bottomStates.AMP)),
                      new InstantCommand(() -> Robot.getArm().setState(Arm.ArmStates.AMP))
              )
      );
      operator.y().onTrue(
              new ParallelCommandGroup(
                      new InstantCommand(() -> Robot.getShooter().setTopState(Shooter.topStates.STAGE)),
                      new InstantCommand(() -> Robot.getShooter().setBottomState(Shooter.bottomStates.STAGE)),
                      new InstantCommand(() -> Robot.getArm().setState(Arm.ArmStates.STAGE))
              )
      );

      operator.b().onTrue(
              new ParallelCommandGroup(
                      new InstantCommand(() -> Robot.getShooter().setTopState(Shooter.topStates.SPEAKER)),
                      new InstantCommand(() -> Robot.getShooter().setBottomState(Shooter.bottomStates.SPEAKER)),
                      new InstantCommand(() -> Robot.getArm().setState(Arm.ArmStates.SPEAKER))
              )
      );

      operator.povDown().onTrue(
              new InstantCommand(Robot.getClimber()::setTarget)
      );

      if(operator.getRightTriggerAxis() > 0){
              new RunCommand(() -> {
                  double intakeInput = IStream.create(operator::getRightTriggerAxis)
                          .filtered(x -> OMath.deadband(x, Constants.Intake.DEADBAND))
                          .get();
                  getIntake().setState(Intake.MotorState.IN);
              }, Robot.getIntake());
              }

      operator.a().onTrue(
              new ParallelCommandGroup(
                      new InstantCommand(() -> Robot.getArm().setState(Arm.ArmStates.FLOOR_INTAKE)),
                      new InstantCommand(() -> Robot.getIntake().setState(Intake.MotorState.IN)).until(() -> Robot.getIntake().isNoteReady())
              )
      );
      driver.povDown().onTrue(
              new ParallelCommandGroup(
                      new InstantCommand(() -> Robot.getShooter().setTopState(Shooter.topStates.OFF))
                              .andThen(new InstantCommand(() -> Robot.getShooter().setBottomState(Shooter.bottomStates.OFF)))
              )
      );

      Robot.getArm().setDefaultCommand(
              new RunCommand(
                      () -> {
                          double armSpeed = IStream.create(operator::getRightY)
                                  .filtered(x -> OMath.deadband(x,0.08))
                                  .get();
                          Robot.getArm().setDesiredTarget(armSpeed);
                      }, Robot.getArm()
              )
      );
  }

      public RobotContainer() {
          configureBindings();
      }

      public Command getAutonomousCommand () {
          return Commands.print("No autonomous command configured");
      }
}
         
