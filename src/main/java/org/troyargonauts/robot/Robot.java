// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.troyargonauts.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import org.troyargonauts.robot.commands.ShootInPlaceAuton;
import org.troyargonauts.robot.commands.StartingSequence;
import org.troyargonauts.robot.generated.TunerConstants;
import org.troyargonauts.robot.subsystems.*;

import com.pathplanner.lib.auto.AutoBuilder;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

/**
 * Class representing the entire robot - CommandBasedRobot framework
 */
public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private SendableChooser<Command> autoChooser;
    private final ScheduledExecutorService scheduledExecutorService = Executors.newSingleThreadScheduledExecutor();

    private static Arm arm;
    private static Climber climber;
    private static Intake intake;
    private static Shooter shooter;
    private static RobotContainer robotContainer;

    private boolean armLimitPressed;

    /**
     * This method is the first that is run when the robot is powered on. Only runs once.
     * Used to instantiate all subsystems, creates a thread for all PID control loops, sets up all available autonomous routines, and the autonomous chooser.
     */
    @Override
    public void robotInit() {
        DataLogManager.start();
        arm = new Arm();
        climber = new Climber();
        intake = new Intake();
        shooter = new Shooter();
      
        robotContainer = new RobotContainer();

        autoChooser = AutoBuilder.buildAutoChooser();
        autoChooser.addOption("ShootInPlace", new ShootInPlaceAuton());
        autoChooser.addOption("Nothing", new WaitCommand(5));
        autoChooser.addOption("StartingSequence", new StartingSequence());
        SmartDashboard.putData("Auto Chooser", autoChooser);

        scheduledExecutorService.scheduleAtFixedRate(() -> {
            if (Robot.getArm().getLimitSwitch()) {
                armLimitPressed = true;
                Robot.getArm().resetEncoders();
            }

            if (armLimitPressed) {
              arm.run();
            }
//            if(robotContainer.getOperatorX()){
//                System.out.println("Xpressed");
//                shooter.run();
//            }
            shooter.run();


            //climber.run();
        }, 100, 10, TimeUnit.MILLISECONDS);
    }

    /**
     * This method runs periodically after the robot has been initialized at start-up. Runs the Command Scheduler.
     */
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        SmartDashboard.putNumber("0", robotContainer.drivetrain.getModule(0).getCANcoder().getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("1", robotContainer.drivetrain.getModule(1).getCANcoder().getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("2", robotContainer.drivetrain.getModule(2).getCANcoder().getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("3", robotContainer.drivetrain.getModule(3).getCANcoder().getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("3 power", robotContainer.drivetrain.getModule(3).getSteerMotor().getMotorVoltage().getValueAsDouble());
    }

    /**
     * This method runs periodically when the robot is in Disabled mode.
     */
    @Override
    public void disabledPeriodic() {}

    /**
     * This method runs once when exiting Disabled mode.
     */
    @Override
    public void disabledInit() {
        //shooter.setDesiredTarget(0,0);
    }

    @Override
    public void disabledExit() {}

    /**
     * This method runs once at the start of Autonomous mode. Selects the autonomous routine to run based on the chooser
     */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = autoChooser.getSelected();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    /**
     * This method runs periodically during Autonomous mode
     */
    @Override
    public void autonomousPeriodic() {}

    /**
     * This method runs once when exiting Autonomous mode.
     */
    @Override
    public void autonomousExit() {}

    /**
     * This method runs once at the start of Teleoperated mode. Cancels any Autonomous command that may be running.
     */
    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    /**
     * This method runs once when exiting Teleoperated mode.
     */
    @Override
    public void teleopExit() {}

    /**
     * This method runs periodically during Teleoperated mode.
     */
    @Override
    public void teleopPeriodic() {
        if(!robotContainer.getOperatorRightBumper() && Robot.getIntake().isNoteReady() && !robotContainer.getOperatorRightTrigger()){
            Robot.getIntake().setState(Intake.IntakeStates.OFF);
            robotContainer.getOperator().getHID().setRumble(GenericHID.RumbleType.kBothRumble,0.5);
            robotContainer.getDriver().getHID().setRumble(GenericHID.RumbleType.kBothRumble,0.5);
        } else{
            robotContainer.getOperator().getHID().setRumble(GenericHID.RumbleType.kBothRumble,0);
            robotContainer.getDriver().getHID().setRumble(GenericHID.RumbleType.kBothRumble,0);

        }
    }

    /**
     * This method runs once at start of Test mode. Cancels all previously running commands.
     */
    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * This method runs once when exiting Test mode.
     */
    @Override
    public void testExit() {}

    /**
     * This method runs periodically during a simulation.
     */
    @Override
    public void simulationPeriodic() {}

    /**
     * Gets the created Shooter subsystem. If a Shooter subsystem has not been created, creates one.
     *
     * @return Shooter Subsystem
     */
    public static Shooter getShooter() {
        if (shooter == null) shooter = new Shooter();
        return shooter;
    }
    /**
     * Gets the created Intake subsystem. If an Intake subsystem has not been created, creates one.
     *
     * @return Intake Subsystem
     */
    public static Intake getIntake() {
        if (intake == null) intake= new Intake();
        return intake;
    }

    /**
     * Gets the created Climber subsystem. If a Climber subsystem has not been created, creates one.
     *
     * @return Climber Subsystem
     */
    public static Climber getClimber() {
        if (climber == null) climber = new Climber();
        return climber;
    }

    /**
     * Gets the created Arm subsystem. If an Arm subsystem has not been created, creates one.
     *
     * @return Arm Subsystem
     */
    public static Arm getArm() {
        if (arm == null) arm = new Arm();
        return arm;
    }
    public static RobotContainer getRobotContainer(){
        return robotContainer;
    }
}
