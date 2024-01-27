package org.troyargonauts.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.troyargonauts.common.motors.wrappers.LazyTalonFX;
import org.troyargonauts.robot.Constants;


public class Shooter extends SubsystemBase {
    private final TalonFX Motor_top, Motor_bottom;

    double Motor_top_target = 0.0;
    double Motor_bottom_target = 0.0;

    public void resetEncoders() {
        Motor_top.setPosition(0);
        Motor_bottom.setPosition(0);
    }
    final VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(0);

    public Shooter(){
        Motor_top = new TalonFX(Constants.Shooter.TOP_MOTOR_ID);
        Motor_bottom = new TalonFX(Constants.Shooter.BOTTOM_MOTOR_ID);
    }

    public void run() {
        Motor_top.setControl(velocityVoltage.withVelocity(Motor_top_target));
        Motor_bottom.setControl(velocityVoltage.withVelocity(Motor_bottom_target));
    }

    public void setDesiredTargetTop(double target) {
            Motor_top_target = target;
        }
    public void setDesiredTargetBottom(double target) {
        Motor_bottom_target = target;
    }

    public void setRawPowerTop(double speed) {
        Motor_top.set(speed);
    }

    public void setRawPowerBottom(double speed) {
        Motor_bottom.set(speed);
    }

    
    public boolean isPidFinishedTop(){
        return (Math.abs((Motor_top_target - Motor_top.getVelocity().getValueAsDouble()) ) <= 5);
    }

    public boolean isPidFinishedBottom(){
        return (Math.abs((Motor_bottom_target - Motor_bottom.getVelocity().getValueAsDouble()) ) <= 5);
    }

}