package org.frc2851.crevolib.drivers;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.InetSocketAddress;

public class JetsonCommunicator implements Runnable {

    static private int port = 9000;
    static private int bufferSize = 128;
    static private byte[] buffer = new byte[bufferSize];
    static private String message = "";

    static private DatagramSocket jetsonSocket;
    static private DatagramPacket packet = new DatagramPacket(buffer, bufferSize);

    private static JetsonCommunicator _instance = new JetsonCommunicator();
    private static Thread _thread = new Thread(_instance);

    private JetsonCommunicator() {
    }

    public void start() {
        try {
            jetsonSocket = new DatagramSocket(new InetSocketAddress(port));
        } catch (java.net.SocketException e) {
            System.out.println("Error: Cannot instantiate server socket!");
            e.printStackTrace();
        }

        _thread.start();
    }

    @Override
    public void run() {
        while (true) {
            try {
                jetsonSocket.receive(packet);
                message = new String(packet.getData(), 0, packet.getLength());
            } catch (java.io.IOException e) {
                System.out.println("Error: Cannot receive packet!");
                e.printStackTrace();
            }
        }
    }

    public static void sendMessage(String message) {
        try {
            jetsonSocket.send(new DatagramPacket(message.getBytes(), message.length()));
        } catch (IOException e) {
            System.out.println("Error: Cannot send message!");
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
        String returnString = "Cannot get this IP!";

        try {
            returnString = InetAddress.getLocalHost().getHostAddress();
        } catch (java.net.UnknownHostException e) {
            System.out.println("Error: Cannot get this IP!");
            e.printStackTrace();
        }

        return returnString;
    }

    public int getThisPort() {
        return port;
    }

}
