package org.frc2851.crevolib.drivers;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import org.frc2851.crevolib.Logger;

/**
 * Creates talons with default configurations
 */

public class TalonSRXFactory
{
    private static final int kMaxRetry = 3;
    private static int talonTimeout = 20;

    private static class Configuration
    {
        public int MAX_OUT = 1;
        public int NOMINAL_OUT = 1;
        public NeutralMode NEUTRAL_MODE = NeutralMode.Brake;
        public boolean ENABLE_CURRENT_LIMIT = false;
        public boolean ENABLE_SOFT_LIMIT = false;
        public int CURRENT_LIMIT = 0;
        public boolean INVERTED = false;

        // Status Frames
        // TODO: Investigate nominal status frame times
        public int CONTROL_FRAME_PERIOD_MS = 10;
        public int STATUS_FRAME_GENERAL_1_MS = 10;
        public int STATUS_FRAME_FEEDBACK0_2_MS = 20;
        public int STATUS_FRAME_QUADRATURE_3_MS = 160;
        public int STATUS_FRAME_ANALOG_4_MS = 160;
        public int STATUS_FRAME_TARGET_10_MS = 0;
        public int STATUS_FRAME_UART_11_MS = 250;
        public int STATUS_FRAME_FEEDBACK_1_MS = 250;
        public int STATUS_FRAME_PIDF0_13_160 = 160;
        public int STATUS_FRAME_PIDF1_14_MS = 250;
        public int STATUS_FRAME_FIRMWARE_15_MS = 160;

        public VelocityMeasPeriod VELOCITY_MEASUREMENT_PERIOD = VelocityMeasPeriod.Period_100Ms;
        public int VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW = 64;
    }

    private static Configuration mDefaultConfiguration = new Configuration();
    private static Configuration mDefaultMasterConfiguration = new Configuration();
    private static Configuration mFastMasterConfiguration = new Configuration();
    private static Configuration mDefaultSlaveConfiguration = new Configuration();

    static
    {
        // Fast Master Configuration
        mFastMasterConfiguration.CONTROL_FRAME_PERIOD_MS = 5;
        mFastMasterConfiguration.STATUS_FRAME_GENERAL_1_MS = 5;
        mFastMasterConfiguration.STATUS_FRAME_FEEDBACK0_2_MS = 10;

        // Slave Configuration
        mDefaultSlaveConfiguration.STATUS_FRAME_GENERAL_1_MS = 100;
        mDefaultSlaveConfiguration.STATUS_FRAME_FEEDBACK0_2_MS = 100;
    }

    /**
     * Creates a standard talon with the default configuration
     * @param id CAN bus id
     * @return Configured talon
     */
    public static TalonSRX createDefaultTalonSRX(int id)
    {
        return createTalonSRX(id, mDefaultConfiguration);
    }

    /**
     * Creates a standard wpi_talon with the default configuration
     * @param id CAN bus id
     * @return Configured talon
     */
    public static WPI_TalonSRX createDefaultWPI_TalonSRX(int id)
    {
        return createWPI_TalonSRX(id, mDefaultConfiguration);
    }

    /**
     * Creates a master talon with the default configuration
     * @param id CAN bus id
     * @return Configured talon
     */
    public static TalonSRX createDefaultMasterTalonSRX(int id)
    {
        return createTalonSRX(id, mDefaultMasterConfiguration);
    }

    /**
     * Creates a master wpi_talon with the default configuration
     * @param id CAN bus id
     * @return Configured talon
     */
    public static WPI_TalonSRX createDefaultMasterWPI_TalonSRX(int id)
    {
        return createWPI_TalonSRX(id, mDefaultMasterConfiguration);
    }

    /**
     * Creates a master talon with the fast configuration
     * @param id CAN bus id
     * @return Configured talon
     */
    public static TalonSRX createFastMasterTalonSRX(int id)
    {
        return createTalonSRX(id, mFastMasterConfiguration);
    }

    /**
     * Creates a master wpi_talon with the fast configuration
     * @param id CAN bus id
     * @return Configured talon
     */
    public static WPI_TalonSRX createFastMasterWPI_TalonSRX(int id)
    {
        return createWPI_TalonSRX(id, mFastMasterConfiguration);
    }

    /**
     * Creates a slave talon with the default configuration
     * @param id CAN bus id
     * @param master The master talon
     * @return Configured talon
     */
    public static TalonSRX createPermanentSlaveTalonSRX(int id, TalonSRX master)
    {
        TalonSRX talon = createTalonSRX(id, mDefaultSlaveConfiguration);
        talon.follow(master);
        return talon;
    }

    /**
     * Creates a slave wpi_talon with the default configuration
     * @param id CAN bus id
     * @param master The master talon
     * @return Configured talon
     */
    public static WPI_TalonSRX createPermanentSlaveWPI_TalonSRX(int id, TalonSRX master)
    {
        WPI_TalonSRX talon = createWPI_TalonSRX(id, mDefaultSlaveConfiguration);
        talon.follow(master);
        return talon;
    }

    public static void configurePIDF(TalonSRX talon, int slot, double p, double i, double d, double f)
    {
        talon.config_kP(slot, p, talonTimeout);
        talon.config_kI(slot, i, talonTimeout);
        talon.config_kD(slot, d, talonTimeout);
        talon.config_kF(slot, f, talonTimeout);
    }

    private static TalonSRX createTalonSRX(int id, Configuration config)
    {
        TalonSRX talon = new TalonSRX(id);
        configureTalon(talon, config);
        return talon;
    }

    private static WPI_TalonSRX createWPI_TalonSRX(int id, Configuration config)
    {
        WPI_TalonSRX talon = new WPI_TalonSRX(id);
        configureTalon(talon, config);
        return talon;
    }

    private static boolean configureTalon(TalonSRX talon, Configuration config)
    {
        boolean setSucceeded = true;
        int retryCounter = 0;

        do {
            setSucceeded &= talon.clearMotionProfileHasUnderrun(talonTimeout) == ErrorCode.OK;
            setSucceeded &= talon.clearStickyFaults(talonTimeout) == ErrorCode.OK;

            setSucceeded &= talon.setControlFramePeriod(ControlFrame.Control_3_General, config.CONTROL_FRAME_PERIOD_MS) == ErrorCode.OK;
            setSucceeded &= talon.setStatusFramePeriod(StatusFrame.Status_1_General, config.STATUS_FRAME_GENERAL_1_MS, talonTimeout) == ErrorCode.OK;
            setSucceeded &= talon.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, config.STATUS_FRAME_FEEDBACK0_2_MS, talonTimeout) == ErrorCode.OK;

            setSucceeded &= talon.setIntegralAccumulator(0, 0, talonTimeout) == ErrorCode.OK;
            setSucceeded &= talon.setIntegralAccumulator(0, 1, talonTimeout) == ErrorCode.OK;
            setSucceeded &= talon.configPeakOutputForward(config.MAX_OUT, talonTimeout) == ErrorCode.OK;
            setSucceeded &= talon.configPeakOutputReverse(-config.MAX_OUT, talonTimeout) == ErrorCode.OK;
            setSucceeded &= talon.configNominalOutputForward(config.NOMINAL_OUT, talonTimeout) == ErrorCode.OK;
            setSucceeded &= talon.configNominalOutputReverse(-config.NOMINAL_OUT, talonTimeout) == ErrorCode.OK;
            setSucceeded &= talon.configContinuousCurrentLimit(config.CURRENT_LIMIT, talonTimeout) == ErrorCode.OK;
            setSucceeded &= talon.configForwardSoftLimitEnable(config.ENABLE_SOFT_LIMIT, talonTimeout) == ErrorCode.OK;
            setSucceeded &= talon.configReverseSoftLimitEnable(config.ENABLE_SOFT_LIMIT, talonTimeout) == ErrorCode.OK;
            setSucceeded &= talon.setSelectedSensorPosition(0, 0, talonTimeout) == ErrorCode.OK;
            //setSucceeded &= talon.setSelectedSensorPosition(0, 1, talonTimeout) == ErrorCode.OK;
            setSucceeded &= talon.configVelocityMeasurementPeriod(config.VELOCITY_MEASUREMENT_PERIOD, talonTimeout) == ErrorCode.OK;
            setSucceeded &= talon.configVelocityMeasurementWindow(config.VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW, talonTimeout) == ErrorCode.OK;
        } while (!setSucceeded && retryCounter++ < kMaxRetry);

        if (!setSucceeded || retryCounter >= kMaxRetry) Logger.println("Failed to initialize Talon " + talon.getDeviceID() + "!!!!", Logger.LogLevel.ERROR);
        return setSucceeded;
    }

    public static void setTalonTimeout(int timeoutMS) { talonTimeout = timeoutMS; }
}