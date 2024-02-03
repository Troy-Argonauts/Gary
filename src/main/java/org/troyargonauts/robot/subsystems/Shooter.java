package org.troyargonauts.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.troyargonauts.common.motors.wrappers.LazyTalonFX;
import org.troyargonauts.robot.Constants;


public class Shooter extends SubsystemBase {
    /**
     * Class representing the Shooter subsystem, including the Data Logging and PID
     *
     * @author aarooshg, TheFlyingPig25
     */
    private TalonFX Motor_top, Motor_bottom;

    double Motor_top_target = 0.0;
    double Motor_bottom_target = 0.0;
    private DoubleLogEntry shooterTopEncoderLog;
    private DoubleLogEntry shooterTopBusVoltage;
    private DoubleLogEntry shooterTopOutputCurrentLog;
    private DoubleLogEntry shooterBottomEncoderLog;
    private DoubleLogEntry shooterBottomBusVoltage;
    private DoubleLogEntry shooterBottomOutputCurrentLog;

    /**
     * Instantiated motor controllers, data logging values and target speed for the Shooter
     * Additionally, we are using PID to set motors to a specific voltage based on RPM in the following method
     * We are also assigning the motor IDs and adding the log entries
     */
    final VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(0);

    public Shooter(){
        Motor_top = new TalonFX(Constants.Shooter.TOP_MOTOR_ID);
        Motor_bottom = new TalonFX(Constants.Shooter.BOTTOM_MOTOR_ID);

        DataLog log = DataLogManager.getLog();
        shooterTopEncoderLog = new DoubleLogEntry((log), "Top Shooter Encoder Values");
        shooterBottomEncoderLog = new DoubleLogEntry((log), "Bottom Shooter Encoder Values");
        shooterTopOutputCurrentLog = new DoubleLogEntry((log), "Top Shooter Motor Output Current ");
        shooterBottomOutputCurrentLog = new DoubleLogEntry((log), "Bottom Shooter Motor Output Current ");
        shooterTopBusVoltage = new DoubleLogEntry((log), "Top Shooter Motor Bus Voltage");
        shooterBottomBusVoltage = new DoubleLogEntry((log), "Bottom Shooter Motor Bus Voltage");
    }

    /**
     * Periodic will constantly check the encoder position, motor voltage, and current output logs
     */
    @Override
    public void periodic(){
        shooterTopEncoderLog.append(Motor_top.getPosition().getValue());
        shooterBottomEncoderLog.append(Motor_bottom.getPosition().getValue());
        shooterTopOutputCurrentLog.append(Motor_top.getSupplyCurrent().getValue());
        shooterTopBusVoltage.append(Motor_top.getMotorVoltage().getValue());
        shooterBottomOutputCurrentLog.append(Motor_bottom.getSupplyCurrent().getValue());
        shooterBottomBusVoltage.append(Motor_bottom.getMotorVoltage().getValue());
    }

    public void run() {
        Motor_top.setControl(velocityVoltage.withVelocity(Motor_top_target));
        Motor_bottom.setControl(velocityVoltage.withVelocity(Motor_bottom_target));
    }

    /**
     * Sets the shooter target speed for both the top and bottom motors
     * @param target
     */
    public void setDesiredTargetTop(double target) {
            Motor_top_target = target;
        }
    public void setDesiredTargetBottom(double target) {
        Motor_bottom_target = target;
    }

    /**
     * Sets the raw power of the two shooter motors
     * @param speed
     */
    public void setRawPowerTop(double speed) {
        Motor_top.set(speed);
    }

    public void setRawPowerBottom(double speed) {
        Motor_bottom.set(speed);
    }
    /**
     * Sets all encoder values to 0.
     */
    public void resetEncoders() {
        Motor_top.setPosition(0);
        Motor_bottom.setPosition(0);
    }

    /**
     * Checks if PID is finished
     * @return the value of the target velocity and actual velocity of both motors
     */
    public boolean isPidFinishedTop(){
        return (Math.abs((Motor_top_target - Motor_top.getVelocity().getValueAsDouble()) ) <= 5);
    }

    public boolean isPidFinishedBottom(){
        return (Math.abs((Motor_bottom_target - Motor_bottom.getVelocity().getValueAsDouble()) ) <= 5);
    }

}