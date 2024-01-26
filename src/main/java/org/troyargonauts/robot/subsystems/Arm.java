package org.troyargonauts.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.troyargonauts.common.motors.wrappers.LazyTalonFX;

public class Arm extends SubsystemBase {

    private LazyTalonFX rightArmMotor;
    private LazyTalonFX leftArmMotor;
    private PIDController armPIDController;

    private double armPosition;
    public Arm(){
        rightArmMotor = new TalonFX(Constants.Arm.PORT);
        leftArmMotor = new TalonFX(Constants.Arm.PORT);
        armPIDController = new PIDController(Constants.Arm.D, Constants.Arm.D, Constants.Arm.D);
        armPosition = 0;
    }

    public void resetEncoders(){
        rightArmMotor.setEncoderCounts(0);
        leftArmMotor.setEncoderCounts(0);
    }

    public void setDesiredState(double position) {
        rightArmMotor.set(armPIDController.calculate(rightArmMotor.getPositionMeters(), position));
        leftArmMotor.set(armPIDController.calculate(leftArmMotor.getPositionMeters(), position));
    }

    public boolean isPIDFinished(double position){
        if (Math.abs(armPosition - position) < 5){
            return true;
        }
        else {
            return false;
        }
    }

        @Override
        public void periodic() {
            armPosition = (leftArmMotor.getPositionRotations() + rightArmMotor.getPositionRotations())/2;

            SmartDashboard.putNumber("Arm Position:", armPosition);
        }
}
