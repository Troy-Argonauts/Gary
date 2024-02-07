// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.troyargonauts.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.troyargonauts.robot.subsystems.*;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private static Shooter shooter;
    private final SendableChooser<Command> chooser = new SendableChooser<>();
    private final ScheduledExecutorService scheduledExecutorService = Executors.newSingleThreadScheduledExecutor();
    private Command autonomousCommand;
  
    private static Arm arm;
    private static Climber climber;
    private static Intake intake;

    @Override
    public void robotInit() {
        LiveWindow.disableAllTelemetry();
        LiveWindow.setEnabled(false);

        DataLogManager.start("/media/sda1/logs");
      
        climber = new Climber();
        intake = new Intake();
        shooter = new Shooter();
        arm = new Arm();
      
        new RobotContainer();
      
        scheduledExecutorService.scheduleAtFixedRate(() -> {
            shooter.run();
            arm.run();
            climber.run();
        }, 100, 10, TimeUnit.MILLISECONDS);
      
        climber.turnDistanceSensorOn();

        CameraServer.startAutomaticCapture().setFPS(14);

        SmartDashboard.putData("Autonomous modes", chooser);
        chooser.addOption("Nothing", new WaitCommand(15));

    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void autonomousInit() {
        autonomousCommand = chooser.getSelected();
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic(){

    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }
  
    public static Shooter getArm() {
        if (shooter == null) shooter = new Shooter();
        return shooter;
    }

    public static Intake getIntake() {
        if (intake == null) intake= new Intake();
        return intake;
    }

    public static Climber getClimber()
    {
        if(climber == null)
        {
            climber = new Climber();
        }
        return climber;
    }

    public static Arm getArm(){
        if (arm == null){
            arm = new Arm();
        }
        return arm;
    }

}
