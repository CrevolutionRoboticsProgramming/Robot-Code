package org.frc2851.crevolib;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import org.jdom2.DataConversionException;
import org.jdom2.Document;
import org.jdom2.Element;
import org.jdom2.JDOMException;
import org.jdom2.input.SAXBuilder;

import java.io.File;
import java.io.IOException;
import java.util.List;

public class ConfigFile
{

    private static SAXBuilder saxBuilder = new SAXBuilder();
    private static Document document;

    private ConfigFile() {}

    /**
     * Imports robot.xml file. Necessary for other functions to work
     */
    public static void readFile()
    {
        File file = new File("/home/lvuser/config/robot.xml");
        try {
            document = saxBuilder.build(file);
        } catch (JDOMException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * @return The name of the robot
     * @throws NullPointerException Thrown if the name attribute is not found in the robot tag
     */
    public static String getRobotName() throws NullPointerException { return document.getRootElement().getAttributeValue("name"); }

    /**
     * Configures and returns a TalonSRX object based on a TalonSRX tag in robot.xml
     * @param name The name of the tag
     * @return TalonSRX object
     * @throws ElementNotFoundException Thrown if the tag is not found in robot.xml
     */
    public static TalonSRX getTalonSRX(String name) throws ElementNotFoundException
    {
        TalonSRX talon = null;
        Element element = getElement("TalonSRX", name);
        int port;
        boolean isInverted = false, usePID = true;
        double p = -1, i = -1, d = -1;
        Element ePid = null;

        try {
            port = element.getAttribute("port").getIntValue();
        } catch (DataConversionException e) {
            System.err.println("TalonSRX [" + name + "] could not configure port");
            return null;
        }

        try {
            isInverted = element.getAttribute("isInverted").getBooleanValue();
        } catch (DataConversionException e) { }

        List<Element> children = element.getChildren();
        for (Element e : children)
            if (e.getName().equals("PID")) ePid = e;

        if (ePid != null)
        {
            try {
                if (element.getAttribute("p") != null) p = element.getAttribute("p").getDoubleValue();
                if (element.getAttribute("i") != null) i = element.getAttribute("i").getDoubleValue();
                if (element.getAttribute("d") != null) i = element.getAttribute("i").getDoubleValue();
            } catch (DataConversionException e) {
                System.out.println("TalonSRX [" + name + "]: Failure to configure PID controller");
                usePID = false;
            }
        }
        
        talon = new TalonSRX(port);
        talon.setInverted(isInverted);
        if (usePID)
        {
            talon.selectProfileSlot(0, 0);
            talon.config_kP(0, p, 0);
            talon.config_kI(0, i, 0);
            talon.config_kD(0, d, 0);
        }

        System.out.println("TalonSRX [" + name + "] was created on port " + port + ":\n\tisInverted: true");
        return talon;
    }

    /**
     * Configures and returns a WPI_TalonSRX object based on a TalonSRX tag in robot.xml
     * @param name The name of the tag
     * @return WPI_TalonSRX object
     * @throws ElementNotFoundException Thrown if the tag is not found in robot.xml
     */
    public static WPI_TalonSRX getWPI_TalonSRX(String name) throws ElementNotFoundException
    {
        WPI_TalonSRX talon;
        Element e = getElement("TalonSRX", name);
        Element ePID = null;
        List<Element> children = e.getChildren();
        int port;
        boolean isInverted = false;
        double kP, kI, kD;

        try {
            port = e.getAttribute("port").getIntValue();
        } catch (DataConversionException e1) {
            Logger.println("TalonSRX [" + name + "]: Could not parse port!", Logger.LogLevel.ERROR);
            throw new ElementNotFoundException();
        }

        try {
            isInverted = e.getAttribute("isInverted").getBooleanValue();
            Logger.println("TalonSRX [" + name + "]: isInverted = " + isInverted, Logger.LogLevel.DEBUG);
        } catch (DataConversionException e1) { }

        talon = new WPI_TalonSRX(port);
        talon.setInverted(isInverted);

        for (Element el : children)
            if (el.getName().equals("PID")) ePID = el;

        if (ePID != null)
        {
            try {
                kP = ePID.getAttribute("p").getDoubleValue();
                kI = ePID.getAttribute("i").getDoubleValue();
                kD = ePID.getAttribute("d").getDoubleValue();

                talon.config_kP(0, kP, 0);
                talon.config_kI(0, kI, 0);
                talon.config_kD(0, kD, 0);
            } catch (DataConversionException e1) {
                Logger.println("TalonSRX [" + name + "]: Failed to parse PID", Logger.LogLevel.ERROR);
            }
        }

        return talon;
    }

    /**
     * Returns integer constant from robot.xml
     * @param name The name of the tag
     * @return The integer value
     * @throws ElementNotFoundException Thrown if the tag is not found in robot.xml
     * @throws DataConversionException Thrown if the value attribute does not contain an integer
     */
    public static int getInt(String name) throws ElementNotFoundException, DataConversionException {
        Element element = getElement("Int", name);
        return element.getAttribute("value").getIntValue();
    }

    /**
     * Returns double constant from robot.xml
     * @param name The name of the tag
     * @return The double value
     * @throws ElementNotFoundException Thrown if the tag is not found in robot.xml
     * @throws DataConversionException Thrown if the value attribute does not contain a double
     */
    public static double getDouble(String name) throws ElementNotFoundException, DataConversionException
    {
        Element element = getElement("Double", name);
        return element.getAttribute("value").getDoubleValue();
    }

    /**
     * Returns boolean constant from robot.xml
     * @param name The name of the tag
     * @return The boolean value
     * @throws ElementNotFoundException Thrown if the tag is not found in robot.xml
     * @throws DataConversionException Thrown if the value attribute does not contain a boolean
     */
    public static boolean getBoolean(String name) throws ElementNotFoundException, DataConversionException
    {
        Element element = getElement("Boolean", name);
        return element.getAttribute("value").getBooleanValue();
    }

    // TODO: [Change]: Added tag filter and removed root element argument
    private static Element getElement(String name, String id) throws ElementNotFoundException
    {
        List<Element> elements = document.getRootElement().getChildren();
        for (Element e : elements)
        {
            if (!e.getName().equals(name)) continue;
            if (e.getAttribute("name").getValue().equals(id)) return e;
        }
        throw new ElementNotFoundException();
    }
}