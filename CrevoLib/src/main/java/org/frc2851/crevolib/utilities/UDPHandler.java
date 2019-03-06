package org.frc2851.crevolib.utilities;

import org.frc2851.crevolib.Logger;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.InetSocketAddress;

public class UDPHandler implements Runnable {

    private int sendPort = 9001;
    private int receivePort = 9000;
    private int bufferSize = 8;
    private byte[] buffer = new byte[bufferSize];
    private String message = "";

    // Static IP of the offboard processor
    private String sendIP = "10.28.51.18";

    private DatagramSocket serverSocket;
    private DatagramPacket packet = new DatagramPacket(buffer, bufferSize);
    private DatagramSocket sendingSocket;

    private static UDPHandler _instance = new UDPHandler();
    private static Thread _thread = new Thread(_instance);

    /**
     * Private constructor ensures that UDPHandler cannot be instantiated outside of the UDPHandler class
     */
    private UDPHandler() {
    }

    /**
     * Starts the processes controlling communication with the Jetson
     */
    public void start() {
        try {
            serverSocket = new DatagramSocket(new InetSocketAddress(receivePort));
            sendingSocket = new DatagramSocket();
        } catch (java.net.SocketException e) {
            Logger.println("Cannot instantiate server socket", Logger.LogLevel.ERROR);
            e.printStackTrace();
        }

        _thread.start();
    }

    /**
     * Receives messages on a separate thread
     */
    @Override
    public void run() {
        while (true) {
            try {
                serverSocket.receive(packet);
                message = new String(packet.getData(), 0, packet.getLength());
            } catch (java.io.IOException e) {
                Logger.println("Cannot receive message", Logger.LogLevel.ERROR);
                e.printStackTrace();
            }
        }
    }

    /**
     * Sends the specified message to the Jetson
     * @param message Message to send to the Jetson
     */
    public void send(String message) {
        try {
            sendingSocket.send(new DatagramPacket(message.getBytes(), message.length(), InetAddress.getByName(sendIP), sendPort));
        } catch (IOException e) {
            Logger.println("Cannot send message", Logger.LogLevel.ERROR);
            e.printStackTrace();
        }
    }

    /**
     * Returns the sole instance of the UDPHandler class
     * @return The instance of the UDPHandler class
     */
    public static UDPHandler getInstance() {
        return _instance;
    }

    /**
     * Returns the message previously received
     * @return The message previously received
     */
    public String getMessage() {
        return message;
    }

    /**
     * Returns the IP of the roboRIO for debugging purposes
     * @return The IP of the roboRIO
     */
    public String getThisIP() {
        String returnString = "Cannot get this IP";

        try {
            returnString = InetAddress.getLocalHost().getHostAddress();
        } catch (java.net.UnknownHostException e) {
            Logger.println("Cannot get this IP", Logger.LogLevel.ERROR);
            e.printStackTrace();
        }

        return returnString;
    }

    /**
     * Returns the port that the roboRIO is receiving messages on
     * @return The port that the roboRIO is receiving messages on
     */
    public int getThisPort() {
        return receivePort;
    }

}
