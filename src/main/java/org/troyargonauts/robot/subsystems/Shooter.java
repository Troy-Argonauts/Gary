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


    final VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(0);

    public void run() {
        Motor_top.setControl(velocityVoltage.withVelocity(Motor_top_target));
        Motor_bottom.setControl(velocityVoltage.withVelocity(Motor_bottom_target));
    }


    public Shooter(){
        Motor_top = new TalonFX(Constants.TOP_MOTOR_ID);
        Motor_bottom = new TalonFX(2);
    }
    public void setDesiredTarget(int motorID, double target){
        if (motorID == 1){
            Motor_top_target = target;
        } else if (motorID == 2){
            Motor_bottom_target = target;
        }
    }
    
    public boolean isPidFinished(int motorID){
        if(motorID == 1){
            return (Math.abs((Motor_top_target - Motor_top.getVelocity().getValueAsDouble()) ) <= 5);
        }else if (motorID == 2){
            return (Math.abs((Motor_bottom_target -  Motor_bottom.getVelocity().getValueAsDouble()) ) <= 5);
        }else {
            return false;
        }
    }

    public void resetEncoders() {
        Motor_top.setPosition(0);
        Motor_bottom.setPosition(0);
    }

    public void setRawPower(int MotorID, double speed) {

        if(MotorID == 1) {
            Motor_top.set(speed);
        } else if (MotorID == 2) {
            Motor_bottom.set(speed);
        }
    }
}