package org.troyargonauts.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.troyargonauts.common.motors.wrappers.LazyTalonFX;
import org.troyargonauts.robot.Constants;


public class Shooter extends SubsystemBase {
    /**
     * Class representing the Shooter subsystem, including the Data Logging and PID
     *
     * @author aarooshg, TheFlyingPig25
     */
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

    /**
     * Instantiated motor controllers, data logging values and target speed for the Shooter
     * Additionally, we are using PID to set motors to a specific voltage based on RPM in the following method
     * We are also assigning the motor IDs and adding the log entries
     */
    final VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(0);

    public Shooter(){
        topMotor = new TalonFX(Constants.Shooter.TOP_MOTOR_ID);
        bottomMotor = new TalonFX(Constants.Shooter.BOTTOM_MOTOR_ID);

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

    public void run() {
        topMotor.setControl(velocityVoltage.withVelocity(topTarget));
        bottomMotor.setControl(velocityVoltage.withVelocity(bottomTarget));
    }

    /**
     * Sets the shooter target speed for both the top and bottom motors
     * @param target
     */
    public void setTopDesiredTarget(double target) {
        topTarget = target;
        }
    public void setBottomDesiredTargetBottom(double target) {
        topTarget = target;
    }

    /**
     * Sets the raw power of the two shooter motors
     * @param power
     */
    public void setTopRawPower(double power) {
        topMotor.set(power);
    }

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
    public enum topSetPoints {
        CENTER(2000),
        STAGE(2000),
        CLOSE(2000),
        OFF(0);

        final double encoderRPM;

        topSetPoints(double encoderRPM) {
            this.encoderRPM = encoderRPM;
        }
    }
        public enum bottomSetPoints{
            CENTER(2000),
            STAGE(2000),
            CLOSE(2000),
            OFF(0);

            final double encoderRPM;

            bottomSetPoints(double encoderRPM) {
                this.encoderRPM = encoderRPM;
            }

        public double getEncoderPosition() {
            return this.encoderRPM;
        }
    }

    /**
     * Checks if PID is finished
     * @return the value of the target velocity and actual velocity of both motors
     */
    public boolean isTopPidFinished(){
        return (Math.abs(topTarget - topEncoderRPM) <= 5);
    }

    public boolean isBottomPidFinished(){
        return (Math.abs(bottomTarget - bottomEncoderRPM) <= 5);
    }
}