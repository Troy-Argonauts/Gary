package org.troyargonauts.robot;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements a swerve subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /**
     * Creates a CommandSwerveDrivetrain object for simulation purposes
     *
     * @param driveTrainConstants
     * @param OdometryUpdateFrequency
     * @param modules
     */
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Creates a CommandSwerveDrivetrain object from sets of expected constants from TunerConstants
     *
     * @param driveTrainConstants
     * @param modules
     */
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        MotorOutputConfigs config = new MotorOutputConfigs();
        for (int i = 0; i < modules.length; i++) {
            var motor = getModule(i).getSteerMotor();
            motor.getConfigurator().refresh(config);
            motor.getConfigurator().apply(config.withDutyCycleNeutralDeadband(0.25));
        }
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Applies a SwerveRequest to the COmmandSwerveDrivetrain to allow for control
     *
     * @param requestSupplier
     * @return
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Starts a simulation thread for the CommandSwerveDrivetrain
     */
    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
}
