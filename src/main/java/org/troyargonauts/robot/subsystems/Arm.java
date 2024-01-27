package org.troyargonauts.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.troyargonauts.robot.Constants.Arm.*;


public class Arm extends SubsystemBase {

    private TalonFX leftArmMotor, rightArmMotor;

    private double leftArmEncoder = 0, rightArmEncoder = 0;

    private double leftArmTarget = 0, rightArmTarget = 0;

    private final PositionVoltage positionVoltage = new PositionVoltage(0).withSlot(0);

    public Arm() {
        leftArmMotor = new TalonFX(LEFT_MOTOR_ID);
        rightArmMotor = new TalonFX(RIGHT_MOTOR_ID);

        leftArmMotor.getConfigurator().apply(new Slot0Configs().withKP(P).withKI(I).withKD(D));
        rightArmMotor.getConfigurator().apply(new Slot0Configs().withKP(P).withKI(I).withKD(D));
    }

    @Override
    public void periodic() {
        leftArmEncoder = leftArmMotor.getPosition().getValueAsDouble();
        rightArmEncoder = rightArmMotor.getPosition().getValueAsDouble();

        SmartDashboard.putNumber("Left Arm Encoder: ", leftArmEncoder);
        SmartDashboard.putNumber("Right Arm Encoder: ", rightArmEncoder);
    }

    public void resetEncoders() {
        leftArmMotor.setPosition(0);
        rightArmMotor.setPosition(0);
    }

    public void run(double target) {
        leftArmMotor.setControl(positionVoltage.withPosition(target));
        rightArmMotor.setControl(positionVoltage.withPosition(target));
    }

    public boolean isPidFinished(int motorID) {
        if (motorID == 1) {
            return (Math.abs((leftArmTarget - leftArmMotor.getVelocity().getValueAsDouble()) ) <= 5);
        } else if (motorID == 2) {
            return (Math.abs((rightArmTarget - rightArmMotor.getVelocity().getValueAsDouble()) ) <= 5);
        } else {
            return false;
        }
    }

}