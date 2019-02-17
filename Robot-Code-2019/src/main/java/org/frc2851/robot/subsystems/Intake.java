package org.frc2851.robot.subsystems;

import badlog.lib.BadLog;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import org.frc2851.crevolib.Logger;
import org.frc2851.crevolib.drivers.TalonSRXFactory;
import org.frc2851.crevolib.io.Button;
import org.frc2851.crevolib.io.Controller;
import org.frc2851.crevolib.subsystem.Command;
import org.frc2851.crevolib.subsystem.Subsystem;
import org.frc2851.robot.Constants;
import org.frc2851.robot.Robot;

public class Intake extends Subsystem {

    Constants mConstants = Constants.getInstance();
    Controller mController = (mConstants.singleControllerMode) ? Robot.driver : Robot.operator;
    WPI_TalonSRX intakeTalon;
    DoubleSolenoid intakeSol;

    private static Intake mInstance = new Intake();

    public static Intake getInstance() {
        return mInstance;
    }
    private Intake() {
        super("Intake");
    }

    private boolean lastDeployState;
    private boolean lastIntakeState, lastOuttakeState;

    void reset() {
        intakeTalon.set(ControlMode.PercentOutput, 0);
        intakeSol.set(DoubleSolenoid.Value.kOff);
    }

    @Override
    protected boolean init(){
        mController.config(Button.ButtonID.Y, Button.ButtonMode.TOGGLE);
        mController.config(Button.ButtonID.RIGHT_BUMPER, Button.ButtonMode.RAW);
        mController.config(Button.ButtonID.LEFT_BUMPER, Button.ButtonMode.RAW);

        intakeTalon = TalonSRXFactory.createDefaultWPI_TalonSRX(mConstants.intakeMaster);
        intakeTalon.setSafetyEnabled(false);

        intakeSol = new DoubleSolenoid(mConstants.pcmID, mConstants.intakeForward, mConstants.intakeReverse);

        BadLog.createTopic("Intake Percent", BadLog.UNITLESS, () -> intakeTalon.getMotorOutputPercent(), "hide", "join:Intake/Percent Outputs");
        BadLog.createTopic("Intake Voltage", "V", () -> intakeTalon.getBusVoltage(), "hide", "join:Intake/Voltage Outputs");
        BadLog.createTopic("Intake Current", "A", () -> intakeTalon.getOutputCurrent(), "hide", "join:Intake/Current Outputs");
        BadLog.createTopic("Hatcher Extended", BadLog.UNITLESS, () -> intakeSol.get() == DoubleSolenoid.Value.kForward ? 1.0 : 0.0, "hide", "join:Intake/Percent Outputs");

        return true;
    }

    @Override
    public Command getDefaultCommand(){
        return new Command() {
            @Override
            public String getName() {
                return "Teleop";
            }

            @Override
            public boolean isFinished() {
                return false;
            }

            @Override
            public boolean init() {
                reset();

                return true;
            }

            @Override
            public void update() {
                // DoubleSolenoid
                if (mController.get(Button.ButtonID.Y)) {
                    intakeSol.set(DoubleSolenoid.Value.kForward);
                    if(!lastDeployState) {
                        log("Intake Deployed", Logger.LogLevel.DEBUG);
                    }
                } else {
                    intakeSol.set(DoubleSolenoid.Value.kReverse);
                }
                lastDeployState = intakeSol.get() == DoubleSolenoid.Value.kForward;

                // Intake
                if (mController.get(Button.ButtonID.RIGHT_BUMPER)) {
                    intakeTalon.set(ControlMode.PercentOutput, 1);
                    log("Intaking", Logger.LogLevel.DEBUG);
                }
                // Outtake
                else if (mController.get(Button.ButtonID.LEFT_BUMPER)) {
                    intakeTalon.set(ControlMode.PercentOutput, -1);
                    log("Outtaking", Logger.LogLevel.DEBUG);
                }
                else {
                    intakeTalon.set(ControlMode.PercentOutput, 0);
                }

                if (mController.get(Button.ButtonID.RIGHT_BUMPER) && !lastIntakeState){
                    log("Began Intaking", Logger.LogLevel.DEBUG);
                } else if (!mController.get(Button.ButtonID.RIGHT_BUMPER) && lastIntakeState){
                    log("Stopped Intaking", Logger.LogLevel.DEBUG);
                }
                lastIntakeState = mController.get(Button.ButtonID.RIGHT_BUMPER);

                if (mController.get(Button.ButtonID.LEFT_BUMPER) && !lastOuttakeState){
                    log("Began Outtaking", Logger.LogLevel.DEBUG);
                } else if (!mController.get(Button.ButtonID.LEFT_BUMPER) && lastOuttakeState){
                    log("Stopped Outtaking", Logger.LogLevel.DEBUG);
                }
                lastOuttakeState = mController.get(Button.ButtonID.LEFT_BUMPER);
            }

            @Override
            public void stop() {
                reset();
            }
        };
    }
}
