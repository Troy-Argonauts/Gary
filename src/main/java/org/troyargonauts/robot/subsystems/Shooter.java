package org.troyargonauts.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.troyargonauts.robot.Constants.Shooter.*;

/**
 * Class representing the Shooter subsystem
 *
 * @author aarooshg, TheFlyingPig25
 */
public class Shooter extends SubsystemBase {
    private TalonFX topMotor, bottomMotor;

    private double topTarget, bottomTarget = 0.0;
    private double topEncoderRPM, bottomEncoderRPM;

    private DoubleLogEntry shooterTopEncoderLog;
    private DoubleLogEntry shooterTopMotorVoltage;
    private DoubleLogEntry shooterTopOutputCurrentLog;
    private DoubleLogEntry shooterBottomEncoderLog;
    private DoubleLogEntry shooterBottomMotorVoltage;
    private DoubleLogEntry shooterBottomOutputCurrentLog;

    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(0);

    /**
     * Instantiates and configures motor controllers and sensors; creates Data Logs. Assigns PID constants.
     */
    public Shooter() {
        topMotor = new TalonFX(TOP_MOTOR_ID, CANBUS_NAME);
        bottomMotor = new TalonFX(BOTTOM_MOTOR_ID, CANBUS_NAME);

        topMotor.getConfigurator().apply(new Slot0Configs().withKP(TOP_MOTOR_P).withKI(TOP_MOTOR_I).withKD(TOP_MOTOR_D));
        bottomMotor.getConfigurator().apply(new Slot0Configs().withKP(BOTTOM_MOTOR_P).withKI(BOTTOM_MOTOR_I).withKD(BOTTOM_MOTOR_D));

//        DataLog log = DataLogManager.getLog();
//
//        shooterTopEncoderLog = new DoubleLogEntry((log), "Top Shooter Encoder Values");
//        shooterBottomEncoderLog = new DoubleLogEntry((log), "Bottom Shooter Encoder Values");
//        shooterTopOutputCurrentLog = new DoubleLogEntry((log), "Top Shooter Motor Output Current ");
//        shooterBottomOutputCurrentLog = new DoubleLogEntry((log), "Bottom Shooter Motor Output Current ");
//        shooterTopMotorVoltage = new DoubleLogEntry((log), "Top Shooter Motor Voltage");
//        shooterBottomMotorVoltage = new DoubleLogEntry((log), "Bottom Shooter Motor Voltage");
    }

    /**
     * Updates the encoder values and outputs their velocities to the SmartDashboard in RPM periodically. Append values to each data log periodically
     */
    @Override
    public void periodic() {
//        shooterTopEncoderLog.append(topMotor.getPosition().getValue());
//        shooterBottomEncoderLog.append(bottomMotor.getPosition().getValue());
//        shooterTopOutputCurrentLog.append(topMotor.getSupplyCurrent().getValue());
//        shooterTopMotorVoltage.append(bottomMotor.getMotorVoltage().getValue());
//        shooterBottomOutputCurrentLog.append(topMotor.getSupplyCurrent().getValue());
//        shooterBottomMotorVoltage.append(bottomMotor.getMotorVoltage().getValue());

        topEncoderRPM = topMotor.getVelocity().getValueAsDouble() * 60;
        bottomEncoderRPM = bottomMotor.getVelocity().getValueAsDouble() * 60;

        SmartDashboard.putNumber("Top Encoder RPM", topEncoderRPM);
        SmartDashboard.putNumber("Bottom Encoder RPM", bottomEncoderRPM);
    }

    /**
     * Sets the PID loops for the top and bottom Shooter motors to their corresponding target velocities
     */
    public void run() {
        topMotor.setControl(velocityVoltage.withVelocity(topTarget / 60));
        bottomMotor.setControl(velocityVoltage.withVelocity(bottomTarget / 60));
    }

    /**
     * Sets the target velocity for the top and bottom motors
     *
     * @param topTarget
     * @param bottomTarget
     */
    public void setDesiredTarget(double topTarget, double bottomTarget) {
        this.topTarget = topTarget;
        this.bottomTarget = bottomTarget;
    }
    
    /**
     * Sets all encoder positions to 0.
     */
    public void resetEncoders() {
        topMotor.setPosition(0);
        bottomMotor.setPosition(0);
    }

    /**
     * Sets enumerators for encoder velocity setpoints of various Shooter States
     */
    public enum ShooterStates {
        /**
         * Shooter off Shooter RPM
         */
        OFF(0, 0),

        /**
         * Amp scoring Shooter RPM
         */
        AMP(1000, 1000),

        /**
         * Podium scoring Shooter RPM
         */
        PODIUM(2000, 2000),

        /**
         * Subwoofer scoring Shooter RPM
         */
        SUBWOOFER(2000, 2000);

        final double encoderTopRPM, encoderBottomRPM;

        ShooterStates(double encoderTopRPM, double encoderBottomRPM) {
            this.encoderTopRPM = encoderTopRPM;
            this.encoderBottomRPM = encoderBottomRPM;
        }
    }

    /**
     * Sets the Shooter motors' targets to the desired Shooter State velocities
     *
     * @param state
     */
    public void setState(ShooterStates state) {
        topTarget = state.encoderTopRPM;
        bottomTarget = state.encoderBottomRPM;
    }

    /**
     * Checks if the PID loop for the top Shooter motor is within the window for its setpoint
     *
     * @return Whether the PID is finished
     */
    public boolean isTopPidFinished() {
        return (Math.abs(topTarget - topEncoderRPM) <= 5);
    }

    /**
     * Checks if the PID loop for the bottom Shooter motor is within the window for its setpoint
     *
     * @return Whether the PID is finished
     */
    public boolean isBottomPidFinished() {
        return (Math.abs(bottomTarget - bottomEncoderRPM) <= 5);
    }
}