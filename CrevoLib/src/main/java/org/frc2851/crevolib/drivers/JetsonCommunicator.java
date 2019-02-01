package org.frc2851.crevolib.drivers;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.InetSocketAddress;

public class JetsonCommunicator implements Runnable {

    private int sendPort = 9001;
    private int receivePort = 9000;
    private int bufferSize = 128;
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
            System.out.println("Error: Cannot instantiate server socket!");
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
                System.out.println("Received!");
            } catch (java.io.IOException e) {
                System.out.println("Error: Could not receive packet!");
                e.printStackTrace();
            }
        }
    }

    public void sendMessage(String message) {
        try {
            sendingSocket.send(new DatagramPacket(message.getBytes(), message.length(), InetAddress.getByName(sendIP), sendPort));
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
        return receivePort;
    }

}
