package org.frc2851.crevolib.drivers;

import org.frc2851.crevolib.Logger;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.InetSocketAddress;

public class JetsonCommunicator implements Runnable {

    private int sendPort = 9001;
    private int receivePort = 9000;
    private int bufferSize = 8;
    private byte[] buffer = new byte[bufferSize];
    private String message = "";

    private String sendIP = "10.0.0.116";

    private DatagramSocket serverSocket;
    private DatagramPacket packet = new DatagramPacket(buffer, bufferSize);
    private DatagramSocket sendingSocket;

    private static JetsonCommunicator _instance = new JetsonCommunicator();
    private static Thread _thread = new Thread(_instance);

    private JetsonCommunicator() {
    }

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

    @Override
    public void run() {
        while (true) {
            try {
                serverSocket.receive(packet);
                message = new String(packet.getData(), 0, packet.getLength());
                System.out.println("Received");
            } catch (java.io.IOException e) {
                Logger.println("Cannot receive message", Logger.LogLevel.ERROR);
                e.printStackTrace();
            }
        }
    }

    public void sendMessage(String message) {
        try {
            sendingSocket.send(new DatagramPacket(message.getBytes(), message.length(), InetAddress.getByName(sendIP), sendPort));
        } catch (IOException e) {
            Logger.println("Cannot send message", Logger.LogLevel.ERROR);
            e.printStackTrace();
        }
    }

    public static JetsonCommunicator getInstance() {
        return _instance;
    }

    public String getMessage() {
        return message;
    }

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

    public int getThisPort() {
        return receivePort;
    }

}
