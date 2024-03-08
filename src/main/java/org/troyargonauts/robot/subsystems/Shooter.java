package org.troyargonauts.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.troyargonauts.robot.Robot;

import static org.troyargonauts.robot.Constants.Shooter.*;

/**
 * Class representing the Shooter subsystem
 *
 * @author aarooshg, TheFlyingPig25
 */
public class Shooter extends SubsystemBase {
    private ShooterStates currentState = ShooterStates.OFF;
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
    private final CoastOut coastRequest = new CoastOut();
//    public final Slot0Configs config = new Slot0Configs();

    /**
     * Instantiates and configures motor controllers and sensors; creates Data Logs. Assigns PID constants.
     */
    public Shooter() {
        topMotor = new TalonFX(TOP_MOTOR_ID, CANBUS_NAME);
        bottomMotor = new TalonFX(BOTTOM_MOTOR_ID, CANBUS_NAME);
//        config.kP = P;
//        config.kI = I;
//        config.kD = D;
//        TalonFXConfiguration allConfigs = new TalonFXConfiguration().withSlot0(config);
        topMotor.setInverted(true);
        bottomMotor.setInverted(true);

//        allConfigs.CurrentLimits.StatorCurrentLimit = 30;

        topMotor.getConfigurator().apply(new Slot0Configs().withKP(P).withKI(I).withKD(D));
        bottomMotor.getConfigurator().apply(new Slot0Configs().withKP(P).withKI(I).withKD(D));
//        topMotor.getConfigurator().apply(allConfigs);
//        bottomMotor.getConfigurator().apply(allConfigs);




        DataLog log = DataLogManager.getLog();
//
//        shooterTopEncoderLog = new DoubleLogEntry((log), "Top Shooter Encoder Values");
//        shooterBottomEncoderLog = new DoubleLogEntry((log), "Bottom Shooter Encoder Values");
        shooterTopOutputCurrentLog = new DoubleLogEntry((log), "Top Shooter Motor Output Current ");
        shooterBottomOutputCurrentLog = new DoubleLogEntry((log), "Bottom Shooter Motor Output Current ");
        shooterTopMotorVoltage = new DoubleLogEntry((log), "Top Shooter Motor Voltage");
        shooterBottomMotorVoltage = new DoubleLogEntry((log), "Bottom Shooter Motor Voltage");

    }

    /**
     * Updates the encoder values and outputs their velocities to the SmartDashboard in RPM periodically. Append values to each data log periodically
     */
    @Override
    public void periodic() {
//        shooterTopEncoderLog.append(topMotor.getPosition().getValue());
//        shooterBottomEncoderLog.append(bottomMotor.getPosition().getValue());
        shooterTopOutputCurrentLog.append(topMotor.getStatorCurrent().getValue());
        shooterTopMotorVoltage.append(bottomMotor.getMotorVoltage().getValue());
        shooterBottomOutputCurrentLog.append(topMotor.getStatorCurrent().getValue());
        shooterBottomMotorVoltage.append(bottomMotor.getMotorVoltage().getValue());

        topEncoderRPM = topMotor.getVelocity().getValueAsDouble() * 60;
        bottomEncoderRPM = bottomMotor.getVelocity().getValueAsDouble() * 60;
        SmartDashboard.putNumber("Top Motor AMP", topMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Bottom Motor AMP", bottomMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Top Encoder RPM", topEncoderRPM);
        SmartDashboard.putBoolean("SHOOTERPID", isTopPidFinished());
        SmartDashboard.putNumber("Bottom Encoder RPM", bottomEncoderRPM);
    }

    public void exit(){
        topMotor.setVoltage(0);
        bottomMotor.setVoltage(0);
    }

    /**
     * Sets the PID loops for the top and bottom Shooter motors to their corresponding target velocities if ShooterState is not OFF. If ShooterState is OFF, motors are set to Coast
     */
    public void run() {
         if(currentState == ShooterStates.OFF){
             topMotor.setControl(coastRequest);
             bottomMotor.setControl(coastRequest);

         } else {
             topMotor.setControl(velocityVoltage.withVelocity(topTarget / 60));
             bottomMotor.setControl(velocityVoltage.withVelocity(bottomTarget / 60));
         }
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
         * Shooter Ramp Up RPM
         */
        RAMPUP(700, 700),
        /**
         * Shooter Idle RPM
         */
        AMP(700, 700),

        /**
         * Amp scoring Shooter RPM
         */
        WING_LINE(4000, 4000),

        /**
         * Stage scoring Shooter RPM
         */
        STAGE(3500, 3500),

        /**
         * Stage scoring Shooter RPM
         */
        WING_NOTE(3000, 3000),

        /**
         * Subwoofer scoring Shooter RPM
         */
        SUBWOOFER(1750, 1750),
        /**
         * ThrowUp Shooter RPM
         */
        THROWOUT(-700,-700);

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
        currentState = state;
    }

    /**
     * Checks if the PID loop for the top Shooter motor is within the window for its setpoint
     *
     * @return Whether the PID is finished
     */
    public boolean isTopPidFinished() {
        return (Math.abs(topTarget - topEncoderRPM) <= 100);
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