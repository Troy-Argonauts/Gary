package org.troyargonauts.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.troyargonauts.common.motors.wrappers.LazyTalonFX;
import org.troyargonauts.robot.Constants;

import static org.troyargonauts.robot.Constants.Shooter.*;

/**
 * Class representing the Shooter subsystem, including the Data Logging and PID
 *
 * @author aarooshg, TheFlyingPig25
 */

public class Shooter extends SubsystemBase {
    private TalonFX topMotor, bottomMotor;

    double topTarget = 0.0;
    double bottomTarget = 0.0;
    private double topEncoderRPM;
    private double bottomEncoderRPM;
    private DoubleLogEntry shooterTopEncoderLog;
    private DoubleLogEntry shooterTopMotorVoltage;
    private DoubleLogEntry shooterTopOutputCurrentLog;
    private DoubleLogEntry shooterBottomEncoderLog;
    private DoubleLogEntry shooterBottomMotorVoltage;
    private DoubleLogEntry shooterBottomOutputCurrentLog;

    final VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(0);

    /**
     * Instantiated motor controllers, data logging values and target speed for the Shooter
     * We are also assigning the motor IDs and adding the log entries
     */
    public Shooter(){
        topMotor = new TalonFX(Constants.Shooter.TOP_MOTOR_ID);
        bottomMotor = new TalonFX(Constants.Shooter.BOTTOM_MOTOR_ID);

        topMotor.getConfigurator().apply(new Slot0Configs().withKP(TOP_MOTOR_P).withKI(TOP_MOTOR_I).withKD(TOP_MOTOR_D));
        bottomMotor.getConfigurator().apply(new Slot0Configs().withKP(BOTTOM_MOTOR_P).withKI(BOTTOM_MOTOR_I).withKD(BOTTOM_MOTOR_D));

        DataLog log = DataLogManager.getLog();
        shooterTopEncoderLog = new DoubleLogEntry((log), "Top Shooter Encoder Values");
        shooterBottomEncoderLog = new DoubleLogEntry((log), "Bottom Shooter Encoder Values");
        shooterTopOutputCurrentLog = new DoubleLogEntry((log), "Top Shooter Motor Output Current ");
        shooterBottomOutputCurrentLog = new DoubleLogEntry((log), "Bottom Shooter Motor Output Current ");
        shooterTopMotorVoltage = new DoubleLogEntry((log), "Top Shooter Motor Voltage");
        shooterBottomMotorVoltage = new DoubleLogEntry((log), "Bottom Shooter Motor Voltage");
    }

    /**
     * Periodic will constantly check the encoder position, motor voltage, and current output logs
     */
    @Override
    public void periodic(){
        shooterTopEncoderLog.append(topMotor.getPosition().getValue());
        shooterBottomEncoderLog.append(bottomMotor.getPosition().getValue());
        shooterTopOutputCurrentLog.append(topMotor.getSupplyCurrent().getValue());
        shooterTopMotorVoltage.append(bottomMotor.getMotorVoltage().getValue());
        shooterBottomOutputCurrentLog.append(topMotor.getSupplyCurrent().getValue());
        shooterBottomMotorVoltage.append(bottomMotor.getMotorVoltage().getValue());

        topEncoderRPM = topMotor.getVelocity().getValueAsDouble();
        bottomEncoderRPM = bottomMotor.getVelocity().getValueAsDouble();

        SmartDashboard.putNumber("Top Encoder Position", topEncoderRPM);
        SmartDashboard.putNumber("Top Encoder Position", bottomEncoderRPM);


    }

    /**
     * Sets the target using a voltage to reach that velocity
     */
    public void run() {
        topMotor.setControl(velocityVoltage.withVelocity(topTarget));
        bottomMotor.setControl(velocityVoltage.withVelocity(bottomTarget));
    }

    /**
     * Sets the shooter target speed for the top motor
     * @param target
     */
    public void setTopDesiredTarget(double target) {
        topTarget = target;
        }
    /**
     * Sets the shooter target speed for the bottom motor
     * @param target
     */
    public void setBottomDesiredTargetBottom(double target) {
        topTarget = target;
    }

    /**
     * Sets the raw power of the top shooter motor
     * @param power
     */
    public void setTopRawPower(double power) {
        topMotor.set(power);
    }

    /**
     * Sets the raw power of the bottom shooter motor
     * @param power
     */
    public void setBottomRawPower(double power) {
        bottomMotor.set(power);
    }
    /**
     * Sets all encoder values to 0.
     */
    public void resetEncoders() {
        topMotor.setPosition(0);
        bottomMotor.setPosition(0);
    }

    /**
     * Creates the top motor states
     */
    public enum topStates {
        STAGE(2000),
        AMP(1000),
        SPEAKER(2000),
        OFF(0);

        final double encoderRPM;

        topStates(double encoderRPM) {
            this.encoderRPM = encoderRPM;
        }
    }
    /**
     * Creates the bottom motor states
     */
        public enum bottomStates{
            STAGE(2000),
            AMP(1000),
            SPEAKER(2000),
            OFF(0);

            final double encoderRPM;

            bottomStates(double encoderRPM) {
                this.encoderRPM = encoderRPM;
            }

        public double getEncoderPosition() {
            return this.encoderRPM;
        }
    }

    /**
     * Sets the top motor to a state determined by the enums
     * @param state
     */
    public void setTopState(topStates state){
        topTarget = state.encoderRPM;
    }
    /**
     * Sets the bottom motor to a state determined by the enums
     * @param state
     */
    public void setBottomState(bottomStates state){
        bottomTarget = state.encoderRPM;
    }

    /**
     * Checks if PID is finished for the top motor
     * @return the value of the target velocity and actual velocity of both motors
     */
    public boolean isTopPidFinished(){
        return (Math.abs(topTarget - topEncoderRPM) <= 5);
    }

    /**
     * Checks if PID is finished for the bottom motor
     * @return the value of the target velocity and actual velocity of both motors
     */
    public boolean isBottomPidFinished(){
        return (Math.abs(bottomTarget - bottomEncoderRPM) <= 5);
    }
}