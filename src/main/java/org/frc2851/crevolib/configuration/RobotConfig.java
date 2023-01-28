package org.frc2851.crevolib.configuration;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import org.frc2851.crevolib.utilities.Logger;
import org.frc2851.crevolib.motion.PID;
import org.jdom2.DataConversionException;
import org.jdom2.Element;

public class RobotConfig extends ConfigFile
{
    /**
     * @return The name of the robot
     * @throws NullPointerException Thrown if the name attribute is not found in the robot tag
     */
    public String getRobotName() throws NullPointerException { return document.getRootElement().getAttributeValue("name"); }

    public TalonSRX getTalon(String name) throws ElementNotFoundException
    {
        Element e = getElement("Talon", name);
        try {
            return new TalonSRX(e.getAttribute("id").getIntValue());
        } catch (DataConversionException exception) {
            Logger.println("Could not create Talon[" + name + "]. Data conversion exception.", Logger.LogLevel.ERROR);
            throw new ElementNotFoundException();
        }
    }

    public WPI_TalonSRX getWpiTalon(String name) throws ElementNotFoundException
    {
        Element e = getElement("Talon", name);
        try {
            return new WPI_TalonSRX(e.getAttribute("id").getIntValue());
        } catch (DataConversionException exception) {
            Logger.println("Could not create Talon[" + name + "]. Data conversion exception.", Logger.LogLevel.ERROR);
            throw new ElementNotFoundException();
        }
    }

    public PID getPID(String name) throws ElementNotFoundException, DataConversionException
    {
        Element e = getElement(name, "PID");
        return new PID(e.getAttribute("p").getDoubleValue(), e.getAttribute("i").getDoubleValue(),
                e.getAttribute("d").getDoubleValue(), e.getAttribute("f").getDoubleValue());
    }

    public int getInt(String name) throws ElementNotFoundException, DataConversionException {
        return getElement(name, "Const").getAttribute("value").getIntValue();
    }

    public double getDouble(String name) throws ElementNotFoundException, DataConversionException {
        return getElement(name, "Const").getAttribute("value").getDoubleValue();
    }

    public boolean getBoolean(String name) throws ElementNotFoundException, DataConversionException {
        return getElement(name, "Const").getAttribute("value").getBooleanValue();
    }

    public String getString(String name) throws ElementNotFoundException, DataConversionException {
        return getElement(name, "Const").getAttribute("value").getValue();
    }

}
