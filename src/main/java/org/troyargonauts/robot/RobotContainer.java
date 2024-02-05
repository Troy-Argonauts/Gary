// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.troyargonauts.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import org.troyargonauts.common.input.Gamepad;
import org.troyargonauts.common.input.gamepads.AutoGamepad;
import org.troyargonauts.common.math.OMath;
import org.troyargonauts.common.streams.IStream;
import org.troyargonauts.robot.subsystems.Arm;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    public static final Gamepad driver = new AutoGamepad(Constants.Controllers.DRIVER);
    public static final Gamepad operator = new AutoGamepad(Constants.Controllers.OPERATOR);


    public RobotContainer() {
        // Configure the button bindings
        configureBindings();
    }

    /**
     * Use this method to define your controller->command mappings.
     */
    private void configureBindings() {

        Robot.getArm().setDefaultCommand(
                new RunCommand(
                        () -> {
                            double joystickAdjust =  IStream.create(operator::getRightY)
                                    .filtered(x -> OMath.deadband(x, Constants.Arm.DEADBAND))
                                    .get();
                            Robot.getArm().adjustSetpoint(joystickAdjust);
                        }
                )
        );


    public static Gamepad getDriver() {
        return driver;
    }

    public static Gamepad getOperator() {
        return operator;
    }
}