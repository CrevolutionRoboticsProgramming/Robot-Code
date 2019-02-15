package org.frc2851.crevolib.drivers;

public class TalonCommunicationErrorException extends Exception {
    private final int port;

    public TalonCommunicationErrorException(int port)
    {
        this.port = port;
    }

    public int getPortNumber()
    {
        return port;
    }
}