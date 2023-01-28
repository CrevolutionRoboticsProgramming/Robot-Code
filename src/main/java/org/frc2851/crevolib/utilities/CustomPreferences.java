package org.frc2851.crevolib.utilities;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.Vector;

public class CustomPreferences
{
    NetworkTable table;
    public CustomPreferences(String name)
    {
        table = NetworkTableInstance.getDefault().getTable(name);
        table.addEntryListener(
                (table, key, entry, label, flag) -> entry.setPersistent(),
                EntryListenerFlags.kImmediate | EntryListenerFlags.kNew);
        table.getEntry(".type").setString("RobotPreferences");
    }

    public void putInt(String key, int value)
    {
        NetworkTableEntry entry = table.getEntry(key);
        entry.setNumber(value);
        entry.setPersistent();
    }

    public void putLong(String key, long value)
    {
        NetworkTableEntry entry = table.getEntry(key);
        entry.setNumber(value);
        entry.setPersistent();
    }

    public void putDouble(String key, double value)
    {
        NetworkTableEntry entry = table.getEntry(key);
        entry.setNumber(value);
        entry.setPersistent();
    }

    public void putFloat(String key, float value)
    {
        NetworkTableEntry entry = table.getEntry(key);
        entry.setNumber(value);
        entry.setPersistent();
    }

    public void putBoolean(String key, boolean value)
    {
        NetworkTableEntry entry = table.getEntry(key);
        entry.setBoolean(value);
        entry.setPersistent();
    }

    public void putString(String key, String value)
    {
        NetworkTableEntry entry = table.getEntry(key);
        entry.setString(value);
        entry.setPersistent();
    }

    public int getInt(String key, int backup)
    {
        return table.getEntry(key).getNumber(backup).intValue();
    }

    public long getLong(String key, long backup)
    {
        return table.getEntry(key).getNumber(backup).longValue();
    }

    public double getDouble(String key, double backup)
    {
        return table.getEntry(key).getNumber(backup).doubleValue();
    }

    public float getFloat(String key, float backup)
    {
        return table.getEntry(key).getNumber(backup).floatValue();
    }

    public boolean getBoolean(String key, boolean backup)
    {
        return table.getEntry(key).getBoolean(backup);
    }

    public String getString(String key, String backup)
    {
        return table.getEntry(key).getString(backup);
    }

    public NetworkTableEntry getEntry(String key)
    {
        return table.getEntry(key);
    }

    public boolean containsKey(String key)
    {
        return table.containsKey(key);
    }

    public void delete(String key)
    {
        table.delete(key);
    }

    public void removeAll()
    {
        for (String key : table.getKeys()) {
            if (!".type".equals(key)) {
                delete(key);
            }
        }
    }

    public Vector<String> getKeys()
    {
        return new Vector<>(table.getKeys());
    }
}
