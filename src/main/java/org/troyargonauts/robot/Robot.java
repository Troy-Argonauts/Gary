// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.troyargonauts.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.troyargonauts.robot.subsystems.*;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final SendableChooser<Command> chooser = new SendableChooser<>();
    private final ScheduledExecutorService scheduledExecutorService = Executors.newSingleThreadScheduledExecutor();

    private static Arm arm;
    private static Climber climber;
    private static Intake intake;
    private static Shooter shooter;
    private static RobotContainer robotContainer;

    private boolean armLimitPressed;

    @Override
    public void robotInit() {
      //  DataLogManager.start("/media/sda1/logs");

        arm = new Arm();
        climber = new Climber();
        intake = new Intake();
        shooter = new Shooter();
      
        robotContainer = new RobotContainer();

        SmartDashboard.putData("Autonomous modes", chooser);

        chooser.setDefaultOption("Nothing", new WaitCommand(15));
        chooser.addOption("Nothing", new WaitCommand(15));
      
        scheduledExecutorService.scheduleAtFixedRate(() -> {
            if (Robot.getArm().getLimitSwitch()) {
                armLimitPressed = true;
                Robot.getArm().resetEncoders();
            }

            if (armLimitPressed) {
              //  arm.run();
            }

            shooter.run();
            //climber.run();
        }, 100, 10, TimeUnit.MILLISECONDS);
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = chooser.getSelected();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        if (robotContainer != null) {
            System.out.println("INit");
        }
    }

    @Override
    public void teleopExit() {}

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
  
    public static Shooter getShooter() {
        if (shooter == null) shooter = new Shooter();
        return shooter;
    }

    public static Intake getIntake() {
        if (intake == null) intake= new Intake();
        return intake;
    }

    public static Climber getClimber() {
        if (climber == null) climber = new Climber();
        return climber;
    }

    public static Arm getArm() {
        if (arm == null) arm = new Arm();
        return arm;
    }
}
