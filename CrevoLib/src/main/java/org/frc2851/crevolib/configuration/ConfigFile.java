package org.frc2851.crevolib.configuration;

import org.frc2851.crevolib.utilities.Logger;
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
    protected static Document document;

    public ConfigFile() {}

    /**
     * Imports robot.xml file. Necessary for other functions to work
     */
    public void readFile(String name)
    {
        File file = new File(name);
        try {
            document = saxBuilder.build(file);
        } catch (JDOMException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    // TODO: [Change]: Added tag filter and removed root element argument
    protected static Element getElement(String name, String id) throws ElementNotFoundException
    {
        List<Element> elements = document.getRootElement().getChildren();
        for (Element e : elements)
        {
            if (!e.getName().equals(name)) continue;
            if (e.getAttribute("name").getValue().equals(id)) return e;
        }
        Logger.println("Could not find element [" + name + "]", Logger.LogLevel.ERROR);
        throw new ElementNotFoundException();
    }
}