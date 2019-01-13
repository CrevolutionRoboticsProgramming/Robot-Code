package org.frc2851.crevolib.utilities;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;

public class TunablePID
{
    private String mName;
    private TalonSRX mTalon;
    private double mSetpoint;
    private boolean mAutoUpdate;
    private int mPort;

    private InetAddress mAddress;
    private DatagramSocket mClientSocket;
    private DatagramPacket mSendPacket, mRecievePacket;
    private byte[] mSendData, mRecieveData;

    public TunablePID(String name, TalonSRX talon, double setpoint, boolean autoUpdate, int port)
    {
        mName = name;
        mTalon = talon;
        mSetpoint = setpoint;
        mAutoUpdate = autoUpdate;
        mPort = port;
    }

    private class UDPSender extends Thread
    {

    }

    private class UDPReciever extends Thread
    {

    }
}
