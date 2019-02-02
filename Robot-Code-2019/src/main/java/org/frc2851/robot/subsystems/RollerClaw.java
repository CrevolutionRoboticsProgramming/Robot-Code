package org.frc2851.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import org.frc2851.crevolib.drivers.TalonSRXFactory;
import org.frc2851.crevolib.subsystem.Command;
import org.frc2851.crevolib.subsystem.Subsystem;
import org.frc2851.robot.Constants;

public class RollerClaw extends Subsystem {
    private WPI_TalonSRX _Motor;
    RollerClaw mInstance = new RollerClaw();
    Constants mConstants = Constants.getInstance();

    private RollerClaw() {
        super("RollerClaw")
    }

    @Override
    protected boolean init() {

        _Motor = TalonSRXFactory.createDefaultMasterWPI_TalonSRX(mConstants.rollerClawTalon);
        return true;
    }


    @Override
    public Command getTeleopCommand() {
        return new Command() {

            @Override
            public String getName() {
                return null;
            }

            @Override
            public boolean isFinished() {
                return false;
            }

            @Override
            public boolean init() {
                return false;
            }

            @Override
            public void update() {

            }

            @Override
            public void stop() {

            }
        };
    }
}
