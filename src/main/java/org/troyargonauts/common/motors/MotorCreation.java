package org.troyargonauts.common.motors;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import edu.wpi.first.wpilibj.DriverStation;
import org.troyargonauts.common.motors.wrappers.LazyCANSparkMax;
import org.troyargonauts.common.motors.wrappers.LazyTalonFX;

public final class MotorCreation {

    private static final int TIMEOUT_MS = 100;

    private MotorCreation() {}

    // Spark Configurations
    public static class SparkConfiguration {
        public boolean BURN_FACTORY_DEFAULT_FLASH;
        public CANSparkMax.IdleMode IDLE_MODE;
        public boolean INVERTED;

        public int STATUS_FRAME_0_RATE_MS;
        public int STATUS_FRAME_1_RATE_MS;
        public int STATUS_FRAME_2_RATE_MS;

        public double OPEN_LOOP_RAMP_RATE;
        public double CLOSED_LOOP_RAMP_RATE;

        public boolean ENABLE_VOLTAGE_COMPENSATION;
        public double NOMINAL_VOLTAGE;
    }

    public static final SparkConfiguration DEFAULT_SPARK_CONFIG = new SparkConfiguration() {
        {
            BURN_FACTORY_DEFAULT_FLASH = false;
            IDLE_MODE = CANSparkMax.IdleMode.kCoast;
            INVERTED = false;
            STATUS_FRAME_0_RATE_MS = 10;
            STATUS_FRAME_1_RATE_MS = 1000;
            STATUS_FRAME_2_RATE_MS = 1000;
            OPEN_LOOP_RAMP_RATE = 0.0;
            CLOSED_LOOP_RAMP_RATE = 0.0;
            ENABLE_VOLTAGE_COMPENSATION = false;
            NOMINAL_VOLTAGE = 12.0;
        }
    };

    private static final SparkConfiguration SLAVE_SPARK_CONFIG = new SparkConfiguration() {
        {
            BURN_FACTORY_DEFAULT_FLASH = false;
            IDLE_MODE = CANSparkMax.IdleMode.kCoast;
            INVERTED = false;
            STATUS_FRAME_0_RATE_MS = 1000;
            STATUS_FRAME_1_RATE_MS = 1000;
            STATUS_FRAME_2_RATE_MS = 1000;
            OPEN_LOOP_RAMP_RATE = 0.0;
            CLOSED_LOOP_RAMP_RATE = 0.0;
            ENABLE_VOLTAGE_COMPENSATION = false;
            NOMINAL_VOLTAGE = 12.0;
        }
    };

    public static LazyCANSparkMax createDefaultSparkMax(int port) {
        return createSparkMax(port, DEFAULT_SPARK_CONFIG);
    }

    private static void handleCANError(final int id, final REVLibError error, final String message) {
        if (error != REVLibError.kOk) {
            DriverStation.reportError("Could not configure spark id: " + id + " error: " + error.toString() + " " + message, false);
        }
    }

    public static LazyCANSparkMax createSparkMax(final int port, final SparkConfiguration config) {
        final LazyCANSparkMax sparkMax = new LazyCANSparkMax(port, CANSparkMax.MotorType.kBrushless);

        handleCANError(port, sparkMax.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, config.STATUS_FRAME_0_RATE_MS), "set status0 rate");
        handleCANError(port, sparkMax.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, config.STATUS_FRAME_1_RATE_MS), "set status1 rate");
        handleCANError(port, sparkMax.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, config.STATUS_FRAME_2_RATE_MS), "set status2 rate");

        handleCANError(port, sparkMax.setIdleMode(config.IDLE_MODE), "set neutral");
        sparkMax.setInverted(config.INVERTED);
        handleCANError(port, sparkMax.setOpenLoopRampRate(config.OPEN_LOOP_RAMP_RATE), "set open loop ramp");
        handleCANError(port, sparkMax.setClosedLoopRampRate(config.CLOSED_LOOP_RAMP_RATE), "set closed loop ramp");

        if (config.ENABLE_VOLTAGE_COMPENSATION) {
            handleCANError(port, sparkMax.enableVoltageCompensation(config.NOMINAL_VOLTAGE), "voltage compensation");
        } else {
            handleCANError(port, sparkMax.disableVoltageCompensation(), "voltage compensation");
        }

        return sparkMax;
    }

    public static class TalonConfiguration {
        public NeutralModeValue NEUTRAL_MODE;
        // factory default
        public double NEUTRAL_DEADBAND;

        public boolean ENABLE_CURRENT_LIMIT;
        public boolean ENABLE_SOFT_LIMIT;
        public boolean ENABLE_LIMIT_SWITCH;
        public int FORWARD_SOFT_LIMIT;
        public int REVERSE_SOFT_LIMIT;

        public InvertedValue INVERTED;
        public boolean SENSOR_PHASE;

        public int CONTROL_FRAME_PERIOD_MS;
        public int MOTION_CONTROL_FRAME_PERIOD_MS;
        public int GENERAL_STATUS_FRAME_RATE_MS;
        public int FEEDBACK_STATUS_FRAME_RATE_MS;
        public int QUAD_ENCODER_STATUS_FRAME_RATE_MS;
        public int ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS;
        public int PULSE_WIDTH_STATUS_FRAME_RATE_MS;

        public SensorVelocityMeasPeriod VELOCITY_MEASUREMENT_PERIOD;
        public int VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW;

        public double OPEN_LOOP_RAMP_RATE;
        public double CLOSED_LOOP_RAMP_RATE;
    }

    private static final TalonConfiguration DEFAULT_TALON_CONFIG = new TalonConfiguration() {
        {
            NEUTRAL_MODE = NeutralModeValue.Coast;
            NEUTRAL_DEADBAND = 0.04;

            ENABLE_CURRENT_LIMIT = false;
            ENABLE_SOFT_LIMIT = false;
            ENABLE_LIMIT_SWITCH = false;
            FORWARD_SOFT_LIMIT = 0;
            REVERSE_SOFT_LIMIT = 0;

            INVERTED = InvertedValue.CounterClockwise_Positive;
            SENSOR_PHASE = false;

            CONTROL_FRAME_PERIOD_MS = 10;
            MOTION_CONTROL_FRAME_PERIOD_MS = 100;
            GENERAL_STATUS_FRAME_RATE_MS = 10;
            FEEDBACK_STATUS_FRAME_RATE_MS = 20;
            QUAD_ENCODER_STATUS_FRAME_RATE_MS = 255;
            ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 255;
            PULSE_WIDTH_STATUS_FRAME_RATE_MS = 255;

            VELOCITY_MEASUREMENT_PERIOD = SensorVelocityMeasPeriod.Period_100Ms;
            VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW = 64;

            OPEN_LOOP_RAMP_RATE = 0.0;
            CLOSED_LOOP_RAMP_RATE = 0.0;
        }
    };


    private static final TalonConfiguration DRIVE_TALON = new TalonConfiguration() {
        {
            NEUTRAL_MODE = NeutralModeValue.Brake;
            NEUTRAL_DEADBAND = 0.04;

            ENABLE_CURRENT_LIMIT = false;
            ENABLE_SOFT_LIMIT = false;
            ENABLE_LIMIT_SWITCH = false;
            FORWARD_SOFT_LIMIT = 0;
            REVERSE_SOFT_LIMIT = 0;

            INVERTED = InvertedValue.CounterClockwise_Positive;
            SENSOR_PHASE = false;

            CONTROL_FRAME_PERIOD_MS = 10;
            MOTION_CONTROL_FRAME_PERIOD_MS = 100;
            GENERAL_STATUS_FRAME_RATE_MS = 10;
            FEEDBACK_STATUS_FRAME_RATE_MS = 20;
            QUAD_ENCODER_STATUS_FRAME_RATE_MS = 255;
            ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 255;
            PULSE_WIDTH_STATUS_FRAME_RATE_MS = 255;

            VELOCITY_MEASUREMENT_PERIOD = SensorVelocityMeasPeriod.Period_25Ms;
            VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW = 8;

            OPEN_LOOP_RAMP_RATE = 0.0;
            CLOSED_LOOP_RAMP_RATE = 0.0;
        }
    };

    public static final TalonConfiguration HIGH_PERFORMANCE_TALON_CONFIG = new TalonConfiguration() {
        {
            NEUTRAL_MODE = NeutralModeValue.Coast;
            NEUTRAL_DEADBAND = 0.02;

            ENABLE_CURRENT_LIMIT = false;
            ENABLE_SOFT_LIMIT = false;
            ENABLE_LIMIT_SWITCH = false;
            FORWARD_SOFT_LIMIT = 0;
            REVERSE_SOFT_LIMIT = 0;

            INVERTED = InvertedValue.CounterClockwise_Positive;
            SENSOR_PHASE = false;

            CONTROL_FRAME_PERIOD_MS = 5;
            MOTION_CONTROL_FRAME_PERIOD_MS = 100;
            GENERAL_STATUS_FRAME_RATE_MS = 5;
            FEEDBACK_STATUS_FRAME_RATE_MS = 10;
            QUAD_ENCODER_STATUS_FRAME_RATE_MS = 255;
            ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 255;
            PULSE_WIDTH_STATUS_FRAME_RATE_MS = 255;

            VELOCITY_MEASUREMENT_PERIOD = SensorVelocityMeasPeriod.Period_10Ms;
            VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW = 32;

            OPEN_LOOP_RAMP_RATE = 0.0;
            CLOSED_LOOP_RAMP_RATE = 0.0;
        }
    };

    private static final TalonConfiguration SLAVE_TALON_CONFIG = new TalonConfiguration() {
        {
            NEUTRAL_MODE = NeutralModeValue.Coast;
            NEUTRAL_DEADBAND = 0.04;

            ENABLE_CURRENT_LIMIT = false;
            ENABLE_SOFT_LIMIT = false;
            ENABLE_LIMIT_SWITCH = false;
            FORWARD_SOFT_LIMIT = 0;
            REVERSE_SOFT_LIMIT = 0;

            INVERTED = InvertedValue.CounterClockwise_Positive;
            SENSOR_PHASE = false;

            CONTROL_FRAME_PERIOD_MS = 10;
            MOTION_CONTROL_FRAME_PERIOD_MS = 100;
            GENERAL_STATUS_FRAME_RATE_MS = 255;
            FEEDBACK_STATUS_FRAME_RATE_MS = 255;
            QUAD_ENCODER_STATUS_FRAME_RATE_MS = 255;
            ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 255;
            PULSE_WIDTH_STATUS_FRAME_RATE_MS = 255;

            VELOCITY_MEASUREMENT_PERIOD = SensorVelocityMeasPeriod.Period_100Ms;
            VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW = 64;

            OPEN_LOOP_RAMP_RATE = 0.0;
            CLOSED_LOOP_RAMP_RATE = 0.0;
        }
    };

    private static void configureTalon(final TalonFX talon, final TalonConfiguration config) {
        talon.set(0.0);

        talon.clearStickyFaults(TIMEOUT_MS);

        TalonFXConfiguration fxConfig = new TalonFXConfiguration();
        fxConfig.MotorOutput.PeakForwardDutyCycle = 1;
        fxConfig.MotorOutput.PeakReverseDutyCycle = -1;

        fxConfig.MotorOutput.NeutralMode = config.NEUTRAL_MODE;

        fxConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = config.FORWARD_SOFT_LIMIT;
        fxConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = config.REVERSE_SOFT_LIMIT;
        fxConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = config.ENABLE_SOFT_LIMIT;
        fxConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = config.ENABLE_SOFT_LIMIT;

        fxConfig.MotorOutput.Inverted = config.INVERTED;

//        talon.setSensorPhase(config.SENSOR_PHASE);

        SlotConfigs slotConfigs = new SlotConfigs();
        slotConfigs.SlotNumber = 0;

//        talon.configVelocityMeasurementPeriod(config.VELOCITY_MEASUREMENT_PERIOD, TIMEOUT_MS);
//        talon.configVelocityMeasurementWindow(config.VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW, TIMEOUT_MS);

        fxConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = config.OPEN_LOOP_RAMP_RATE;
        fxConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = config.CLOSED_LOOP_RAMP_RATE;

        fxConfig.Voltage.SupplyVoltageTimeConstant = 32;


//        talon.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero, TIMEOUT_MS);
        fxConfig.Audio.BeepOnBoot = false;

//        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, config.GENERAL_STATUS_FRAME_RATE_MS, TIMEOUT_MS);
//        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, config.FEEDBACK_STATUS_FRAME_RATE_MS, TIMEOUT_MS);
//        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, config.QUAD_ENCODER_STATUS_FRAME_RATE_MS, TIMEOUT_MS);
//        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, config.ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS, TIMEOUT_MS);
//        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, config.PULSE_WIDTH_STATUS_FRAME_RATE_MS, TIMEOUT_MS);
//
//        talon.setControlFramePeriod(ControlFrame.Control_3_General, config.CONTROL_FRAME_PERIOD_MS);
    }

    public static LazyTalonFX<TalonFX> createDefaultTalonFX(final int canID) {
        TalonFX talon = new TalonFX(canID);
        configureTalon(talon, DEFAULT_TALON_CONFIG);
//        talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, TIMEOUT_MS);
        return new LazyTalonFX<>(talon);
    }

    public static LazyTalonFX<TalonFX> createTalonFX(final int canID, final TalonConfiguration configuration, final boolean isSlave) {
        TalonFX talon = new TalonFX(canID);
        configureTalon(talon, configuration);
//        talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, TIMEOUT_MS);
        return new LazyTalonFX<>(talon);
    }

    public static LazyTalonFX<TalonFX> createDriveTalonFX(final int canID, final boolean isSlave) {
        return createTalonFX(canID, DRIVE_TALON, isSlave);
    }
}

