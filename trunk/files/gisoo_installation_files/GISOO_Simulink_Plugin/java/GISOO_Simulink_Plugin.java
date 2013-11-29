/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

/**
 *
 * @author behdad
 */
import javax.swing.BorderFactory;
import javax.swing.Box;
import javax.swing.JComponent;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTextField;
import javax.swing.JCheckBox;

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.GridLayout;

import org.apache.log4j.Logger;
import org.jdom.Element;

import se.sics.cooja.ClassDescription;
import se.sics.cooja.GUI;
import se.sics.cooja.Mote;
import se.sics.cooja.MotePlugin;
import se.sics.cooja.PluginType;
import se.sics.cooja.Simulation;
import se.sics.cooja.VisPlugin;
import se.sics.cooja.interfaces.SerialPort;
import se.sics.cooja.TimeEvent;

import java.util.Collection;
import java.util.Observable;
import java.util.Observer;

import se.sics.cooja.mspmote.SkyMote;
import se.sics.mspsim.core.ADC12;
import se.sics.mspsim.core.ADCInput;
import se.sics.mspsim.core.IOUnit;
import se.sics.mspsim.core.DAC12;
import se.sics.mspsim.core.DACOutput;

//UDP Sending
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;

//For Writing in a text file
import java.io.*;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

@ClassDescription("GISOO-Simulink Plugin")
@PluginType(PluginType.MOTE_PLUGIN)
public class GISOO_Simulink_Plugin extends VisPlugin implements MotePlugin {

    private static Logger logger = Logger.getLogger(GISOO_Simulink_Plugin.class);
    private Simulation sim;
    private Mote mote;
    private SkyMote skyMote;
    public final int LISTEN_PORT;
    private final int SIMULINK_PORT = 55555;
    private DatagramSocket dSocket;
    private DatagramPacket dPacket;
    private int simulationTime;
    private byte[] buffer = new byte[4];
    private JCheckBox serialDataShouldRCV;
    private JTextField CPUDellay;
    private final static int LABEL_WIDTH = 60;
    private final static int LABEL_HEIGHT = 15;
    private JLabel statusLabel, ii;
    private File adcFile = new File("./../apps/GISOO_Simulink_Plugin/logs/ADCTime.txt");
    private File dacFile = new File("./../apps/GISOO_Simulink_Plugin/logs/DACTime.txt");
    private File serialFileSend = new File("./../apps/GISOO_Simulink_Plugin/logs/SerialTimeSend.txt");
    private File serialFileRCV = new File("./../apps/GISOO_Simulink_Plugin/logs/SerialTimeRCV.txt");
    private boolean DACWrittenForFirst = true;
    private boolean ADCWrittenForFirst = true;
    private boolean SerialRCVWrittenForFirst = true;
    private boolean SerialSendWrittenForFirst = true;
    private long lastTimeForFile = 0;
    private int serialPinNumber;
    private SerialPort serialPort;
    private Observer serialDataObserver;
    int nRCVedByte = 0;
    int sizeOfPayload = 0;
    int messageIndex = 0;
    byte lastByte;
    byte[] message = new byte[28];
    final byte[] serialPayload = new byte[16];
    boolean serialSendFlag;

    protected class ADCConnector implements ADCInput {

        private int pinNumber;
        private int motePinId;

        public ADCConnector(int pin) {
            pinNumber = pin;
        }

        @Override
        public int nextData() {

            //Simulation Time
            simulationTime = (int) sim.getSimulationTimeMillis();
//            logger.info("Simulation time is: " + simulationTime);

            //MotePinId
            motePinId = (skyMote.getID() * 100) + pinNumber;
//            logger.info("motePinId is: " + motePinId);

            byte[] message = adcRequestMsgCreator(simulationTime, motePinId);
            int sensorValue = adcRequestSender(message, dSocket);
//            logger.info("Sensor Value is: " + sensorValue);
            fileWriter(motePinId, sensorValue);
            return (sensorValue);

        }

        protected int adcRequestSender(byte[] message, DatagramSocket dgSocket) {

            try {
                InetAddress IPAddress = InetAddress.getByName("localhost");
                DatagramPacket dgPacket = new DatagramPacket(message, message.length, IPAddress, SIMULINK_PORT);
                dgSocket.send(dgPacket);
            } catch (IOException ex) {
                System.out.println("Error in sending dgPacket in adcRequestSender!!");
            }
            byte[] receivedValue = new byte[4];
            DatagramPacket receivedPacket = new DatagramPacket(receivedValue, receivedValue.length);
            try {
                dgSocket.receive(receivedPacket);
            } catch (IOException ex) {
                System.out.println("Error in receive UDP data adcRequestSender!!");
                System.out.println(ex.getMessage());
            }

            byte[] data = receivedPacket.getData();
            return (bytes2Int1(data));
        }

        int bytes2Int1(byte[] bytes) {
            return ((int) (0xff & bytes[0]) << 24
                    | (int) (0xff & bytes[1]) << 16
                    | (int) (0xff & bytes[2]) << 8
                    | (int) (0xff & bytes[3]) << 0);
        }

        protected byte[] adcRequestMsgCreator(int simulationTime, int motePinId) {
            int uValue = 0;
            byte[] emptySerialData = new byte[16];
            for (byte b : emptySerialData) {
                b = 0;
            }

            byte[] simulationTimeInByte = int2Bytes(simulationTime);
            byte[] motePinIdInByte = int2Bytes(motePinId);
            byte[] uValueInByte = int2Bytes1(uValue);

            byte[] message = combine1(reverseArray(simulationTimeInByte), reverseArray(motePinIdInByte), reverseArray(uValueInByte), emptySerialData);

            return (message);
        }

        protected byte[] combine1(byte[] one, byte[] two) {
            byte[] combined = new byte[one.length + two.length];

            System.arraycopy(one, 0, combined, 0, one.length);
            System.arraycopy(two, 0, combined, one.length, two.length);
            return combined;
        }

        protected byte[] combine1(byte[] one, byte[] two, byte[] three) {
            byte[] combined = new byte[one.length + two.length + three.length];

            System.arraycopy(one, 0, combined, 0, one.length);
            System.arraycopy(two, 0, combined, one.length, two.length);
            System.arraycopy(three, 0, combined, one.length + two.length, three.length);
            return combined;
        }

        protected byte[] combine1(byte[] one, byte[] two, byte[] three, byte[] four) {
            byte[] combined = new byte[one.length + two.length + three.length + four.length];

            System.arraycopy(one, 0, combined, 0, one.length);
            System.arraycopy(two, 0, combined, one.length, two.length);
            System.arraycopy(three, 0, combined, one.length + two.length, three.length);
            System.arraycopy(four, 0, combined, one.length + two.length + three.length, four.length);
            return combined;
        }

        protected byte[] int2Bytes1(int inputInt) {

            byte[] resultByts = new byte[]{
                (byte) ((inputInt >> 24) & 0xff),
                (byte) ((inputInt >> 16) & 0xff),
                (byte) ((inputInt >> 8) & 0xff),
                (byte) ((inputInt >> 0) & 0xff),};
            return resultByts;
        }
    }

    protected class DACConnector implements DACOutput {

        private int pinNumber;
        private int motePinId;

        public DACConnector(int pin) {

            pinNumber = pin;
        }

        public void setDACPin0(int value) {
            simulationTime = (int) sim.getSimulationTimeMillis();
            motePinId = (skyMote.getID() * 100) + 16;// 16 = pin number for DAC0
            byte[] message = dacMessageCreator(simulationTime, motePinId, value);
            int confirmedAppliedUValue = dacValueSender(message, dSocket);
            fileWriter(motePinId, confirmedAppliedUValue);
//                logger.info("DAC Value:" + confirmedAppliedUValue );
        }

        public void setDACPin1(int value) {
            simulationTime = (int) sim.getSimulationTimeMillis();
            motePinId = (skyMote.getID() * 100) + 17;//17 = pin number for ADC1

            byte[] message = dacMessageCreator(simulationTime, motePinId, value);
            int u = value;
            int confirmedAppliedUValue = dacValueSender(message, dSocket);
            fileWriter(motePinId, confirmedAppliedUValue);
//                logger.info("DAC Value:" + confirmedAppliedUValue );
        }

        byte[] dacMessageCreator(int simulationTime, int motePinId, int uValue) {
            byte[] emptySerialData = new byte[16];
            for (byte b : emptySerialData) {
                b = 0;
            }

            byte[] simulationTimeInByte = dacInt2Bytes(simulationTime);
            byte[] motePinIdInByte = dacInt2Bytes(motePinId);
            byte[] uValueInByte = dacInt2Bytes(uValue);
            byte[] message = dacCombine(reverseArray(simulationTimeInByte), reverseArray(motePinIdInByte), reverseArray(uValueInByte), emptySerialData);
            return (message);
        }

        int dacValueSender(byte[] message, DatagramSocket dgSocket) {
            try {
                InetAddress IPAddress = InetAddress.getByName("localhost");
                DatagramPacket dgPacket = new DatagramPacket(message, message.length, IPAddress, SIMULINK_PORT);
                dgSocket.send(dgPacket);
            } catch (IOException ex) {
                System.out.println("Error in sending dgPacket in adcRequestSender!!");
            }
            byte[] receivedValue = new byte[4];
            DatagramPacket receivedPacket = new DatagramPacket(receivedValue, receivedValue.length);
            try {
                dgSocket.receive(receivedPacket);
            } catch (IOException ex) {
                System.out.println("Error in receive UDP data adcRequestSender!!");
                System.out.println(ex.getMessage());
            }
            byte[] data = receivedPacket.getData();
            return (dacBytes2Int(data));
        }

        byte[] dacCombine(byte[] one, byte[] two, byte[] three, byte[] four) {
            byte[] combined = new byte[one.length + two.length + three.length + four.length];
            System.arraycopy(one, 0, combined, 0, one.length);
            System.arraycopy(two, 0, combined, one.length, two.length);
            System.arraycopy(three, 0, combined, one.length + two.length, three.length);
            System.arraycopy(four, 0, combined, one.length + two.length + three.length, four.length);
            return combined;
        }

        byte[] dacInt2Bytes(int inputInt) {
            byte[] resultByts = new byte[]{
                (byte) ((inputInt >> 24) & 0xff),
                (byte) ((inputInt >> 16) & 0xff),
                (byte) ((inputInt >> 8) & 0xff),
                (byte) ((inputInt >> 0) & 0xff),};
            return resultByts;
        }

        int dacBytes2Int(byte[] bytes) {
            return ((int) (0xff & bytes[0]) << 24
                    | (int) (0xff & bytes[1]) << 16
                    | (int) (0xff & bytes[2]) << 8
                    | (int) (0xff & bytes[3]) << 0);
        }
    }

    public GISOO_Simulink_Plugin(Mote mote, Simulation simulation, final GUI gui) {

        super("GISOO-Simulink Plugin (" + mote + ")", gui, false);
        this.mote = mote;
        skyMote = (SkyMote) mote;
        sim = skyMote.getSimulation();
        LISTEN_PORT = 18000 + mote.getID();

        if (GUI.isVisualized()) {
            this.getContentPane().setSize(100, 100);
            Box northBox = Box.createHorizontalBox();
            northBox.setBorder(BorderFactory.createEmptyBorder(0, 5, 5, 5));
            JPanel smallPanel = new JPanel(new BorderLayout());
            JLabel label = new JLabel("Listening on port: " + LISTEN_PORT);
            label.setPreferredSize(new Dimension(LABEL_WIDTH, 20));
            smallPanel.add(BorderLayout.CENTER, label);
            northBox.add(smallPanel);

            Box mainBox = Box.createHorizontalBox();
            mainBox.setBorder(BorderFactory.createEmptyBorder(0, 5, 5, 5));
            JPanel smallPane = new JPanel(new BorderLayout());
            JLabel simulinkPort = new JLabel("Sending data to port: " + SIMULINK_PORT);
            simulinkPort.setPreferredSize(new Dimension(LABEL_WIDTH, LABEL_HEIGHT));
            smallPane.add(BorderLayout.CENTER, simulinkPort);
            mainBox.add(smallPane);

            Box southernBox = Box.createHorizontalBox();
            southernBox.setBorder(BorderFactory.createEmptyBorder(0, 5, 5, 5));
            JPanel smallPane2 = new JPanel(new BorderLayout());
            serialDataShouldRCV = new JCheckBox("Serial reply should be received after(ms): ");
            CPUDellay = new JTextField("1");
            CPUDellay.setPreferredSize(new Dimension(LABEL_WIDTH, 25));
            smallPane2.add(BorderLayout.WEST, serialDataShouldRCV);
            smallPane2.add(BorderLayout.CENTER, CPUDellay);
            southernBox.add(smallPane2);

            getContentPane().add(BorderLayout.NORTH, northBox);
            getContentPane().add(BorderLayout.CENTER, mainBox);
            getContentPane().add(BorderLayout.SOUTH, southernBox);
            pack();

        }

        /* Mote serial port */
        serialPort = (SerialPort) mote.getInterfaces().getLog();
        if (serialPort == null) {
            throw new RuntimeException("No mote serial port");
        }
        for (byte b : serialPayload) {
            b = 0;
        }

        logger.info("Listening on port: " + LISTEN_PORT);


        /* Observe serial port for outgoing data */
        serialPort.addSerialDataObserver(serialDataObserver = new Observer() {
            public void update(Observable obs, Object obj) {
                serialSendFlag = false;
                /* Check the received message to find the moment that message follow the header format
                 "0x 00 ff ff 00 00" which is equal to "0 -1 -1 0 0" in decimal. The next byte after 
                 this message is the message size*/

                lastByte = serialPort.getLastSerialData();
                logger.info("lastByte: " + lastByte);
                if (lastByte == 126 && nRCVedByte < 7) {
                    nRCVedByte = 1;
                    //  messageIndex=0;
                } else if (nRCVedByte > 0 && nRCVedByte < 7) {
                    nRCVedByte++;
                } else if (nRCVedByte == 7) {
                    sizeOfPayload = lastByte;
//                    logger.info("sizeOfPayload is: " + sizeOfPayload);
                    nRCVedByte++;
                } else if (nRCVedByte > 7 && nRCVedByte < 10) {
                    nRCVedByte++;
                } else if (nRCVedByte >= 10 && nRCVedByte < 10 + sizeOfPayload) {
//                    logger.info("nRCVedByte: " + nRCVedByte);
//                    logger.info("messageIndex: " + messageIndex);

                    serialPayload[messageIndex] = lastByte;
//                    logger.info("messageIndex: " + messageIndex + "   lastByte: " + lastByte);
                    if (nRCVedByte == (10 + sizeOfPayload - 1)) {
                        messageIndex = 0;
                        serialSendFlag = true;
                    } else {
                        messageIndex++;
                    }
                    nRCVedByte++;
                } else {
                    nRCVedByte = 0;
                }

                if (serialSendFlag) {

                    simulationTime = (int) sim.getSimulationTimeMillis();
                    serialPinNumber = (skyMote.getID() * 100) + 18;//18 = pinNumber for Sending data to the serial port in Simulink
                    int u = 0;//value;
                    byte[] serialDataMSG = serialMessageCreator(simulationTime, serialPinNumber, serialPayload);
                    fileWriter(serialPinNumber, serialPayload);
                    int q = serialMsgSender(serialDataMSG, dSocket);
                    serialSendFlag = false;
                    if (serialDataShouldRCV.isSelected()) //If the second scenario has been selected (It should check the second scenario CheckBox )
                    {
//                        logger.info("Simulation Time is : " + sim.getSimulationTime());
                        TimeEvent delayedEvent = new TimeEvent(0) {
                            public void execute(long t) {
//                                logger.info("  Simulation Time is : " + sim.getSimulationTime());


                                simulationTime = (int) sim.getSimulationTimeMillis();
                                //                              logger.info("When recoded Simulation Time is : " + sim.getSimulationTime());
                                serialPinNumber = (skyMote.getID() * 100) + 19;//SerialDataRequest;
                                //                            logger.info("SerialPinNumber is : " + serialPinNumber);

                                byte[] serialRequestMesg = serialRequestMsgCreator(simulationTime, serialPinNumber);
                                serialRequestSender(serialRequestMesg, dSocket);
                                fileWriter(serialPinNumber, serialRequestMesg);
                            }
                        };

                        sim.scheduleEvent(
                                delayedEvent,
                                sim.getSimulationTime() + (Long.parseLong(CPUDellay.getText()) * (sim.MILLISECOND)));
                    }
                }
            }
        });

        try {
            dSocket = new DatagramSocket(LISTEN_PORT);//(18000 + skyMote.getID());

        } catch (SocketException ex) {
            System.out.println("Errore in dSocket creation");
        }
        IOUnit adc = skyMote.getCPU().getIOUnit("ADC12");
        if (adc instanceof ADC12) {
            ((ADC12) adc).setADCInput(0, new ADCConnector(0));
            ((ADC12) adc).setADCInput(1, new ADCConnector(1));
            ((ADC12) adc).setADCInput(2, new ADCConnector(2));
            ((ADC12) adc).setADCInput(3, new ADCConnector(3));
            ((ADC12) adc).setADCInput(4, new ADCConnector(4));
            ((ADC12) adc).setADCInput(5, new ADCConnector(5));
            ((ADC12) adc).setADCInput(6, new ADCConnector(6));
            ((ADC12) adc).setADCInput(7, new ADCConnector(7));
            ((ADC12) adc).setADCInput(8, new ADCConnector(8));
            ((ADC12) adc).setADCInput(9, new ADCConnector(9));
            ((ADC12) adc).setADCInput(10, new ADCConnector(10));
            ((ADC12) adc).setADCInput(11, new ADCConnector(11));
            ((ADC12) adc).setADCInput(12, new ADCConnector(12));
            ((ADC12) adc).setADCInput(13, new ADCConnector(13));
            ((ADC12) adc).setADCInput(14, new ADCConnector(14));
            ((ADC12) adc).setADCInput(15, new ADCConnector(15));
        }
        ((DAC12) skyMote.getCPU().getIOUnit("DAC12")).setDACOutput(0, new DACConnector(16));
        ((DAC12) skyMote.getCPU().getIOUnit("DAC12")).setDACOutput(1, new DACConnector(17));
    }

    void fileWriter(int motePinId, int value) {

        //Writing the Simulation time and Sensor value in a file
        int pinId = 0;
        pinId = motePinId % 100;
        String content = "";
        String content1 = ("motePinId" + '\t' + '\t' + "SimulationTime" + '\t' + '\t' + "value");

        if (pinId == 0 || pinId == 1) {
            try {
//                String content = "";
                content = (String.valueOf(motePinId) + '\t' + '\t' + String.valueOf(sim.getSimulationTimeMillis()) + '\t' + '\t' + String.valueOf(value));

                lastTimeForFile = sim.getSimulationTimeMillis();

                // if file doesn't exists, then create it
                if (!adcFile.exists()) {
                    adcFile.createNewFile();
                } else if (ADCWrittenForFirst) {
                    adcFile.delete();
                    adcFile.createNewFile();

                    FileWriter fw1 = new FileWriter(adcFile.getAbsoluteFile(), true);
                    BufferedWriter bw1 = new BufferedWriter(fw1);
                    PrintWriter pn1 = new PrintWriter(fw1);
                    pn1.println(content1);
                    pn1.close();

                    ADCWrittenForFirst = false;
                }

                FileWriter fw = new FileWriter(adcFile.getAbsoluteFile(), true);
                BufferedWriter bw = new BufferedWriter(fw);
                PrintWriter pn = new PrintWriter(fw);
                pn.println(content);
                pn.close();
//               logger.info("Done");
            } catch (Exception e) {//Catch exception if any
                logger.info("Error in print dacFile");
            }
        } else if (pinId == 16 || pinId == 17) {
            try {
                content = (String.valueOf(motePinId) + '\t' + '\t' + String.valueOf(sim.getSimulationTimeMillis()) + '\t' + '\t' + String.valueOf(value));
                if (!dacFile.exists()) {
                    dacFile.createNewFile();
                } else if (DACWrittenForFirst) {
                    dacFile.delete();
                    dacFile.createNewFile();

                    FileWriter fw1 = new FileWriter(dacFile.getAbsoluteFile(), true);
                    BufferedWriter bw1 = new BufferedWriter(fw1);
                    PrintWriter pn1 = new PrintWriter(fw1);
                    pn1.println(content1);
                    pn1.close();

                    DACWrittenForFirst = false;
                }

                FileWriter fw = new FileWriter(dacFile.getAbsoluteFile(), true);

                BufferedWriter bw = new BufferedWriter(fw);
                PrintWriter pn = new PrintWriter(fw);
                pn.println(content);
                pn.close();
//                        logger.info("Done");
            } catch (Exception e) {//Catch exception if any
                logger.info("Error in print dacFile");
            }
        }
        //End of writing
    }

    void fileWriter(int motePinId, byte[] message) {

        //Writing the Simulation time and Sensor value in a file
        int pinId = 0;
        pinId = motePinId % 100;
        String content = "";

        if (pinId == 18) {
            try {
//                String content = "";
                content = (String.valueOf(motePinId) + '\t' + '\t' + String.valueOf(sim.getSimulationTimeMillis()) + '\t' + '\t'
                        + String.valueOf(message[0]) + '\t' + '\t' + String.valueOf(message[1]) + '\t' + '\t' + String.valueOf(message[2]) + '\t' + '\t' + String.valueOf(message[3]) + '\t' + '\t'
                        + String.valueOf(message[4]) + '\t' + '\t' + String.valueOf(message[5]) + '\t' + '\t' + String.valueOf(message[6]) + '\t' + '\t' + String.valueOf(message[7]) + '\t' + '\t'
                        + String.valueOf(message[8]) + '\t' + '\t' + String.valueOf(message[9]) + '\t' + '\t' + String.valueOf(message[10]) + '\t' + '\t' + String.valueOf(message[11]) + '\t' + '\t'
                        + String.valueOf(message[12]) + '\t' + '\t' + String.valueOf(message[13]) + '\t' + '\t' + String.valueOf(message[14]) + '\t' + '\t' + String.valueOf(message[15]));


                lastTimeForFile = sim.getSimulationTimeMillis();

                // if file doesn't exists, then create it
                if (!serialFileSend.exists()) {
                    serialFileSend.createNewFile();
                } else if (SerialSendWrittenForFirst) {
                    serialFileSend.delete();
                    serialFileSend.createNewFile();
                    SerialSendWrittenForFirst = false;
                }

                FileWriter fw = new FileWriter(serialFileSend.getAbsoluteFile(), true);

                BufferedWriter bw = new BufferedWriter(fw);
                PrintWriter pn = new PrintWriter(fw);
                pn.println(content);
                pn.close();

                logger.info("Done");
            } catch (Exception e) {//Catch exception if any
                logger.info("Error in print dacFile");
            }
        } else if (pinId == 19) {
            try {
//                String content = "";
                content = (String.valueOf(motePinId) + '\t' + '\t' + String.valueOf(sim.getSimulationTimeMillis()) + '\t' + '\t'
                        + String.valueOf(message[0]) + '\t' + '\t' + String.valueOf(message[1]) + '\t' + '\t' + String.valueOf(message[2]) + '\t' + '\t' + String.valueOf(message[3]) + '\t' + '\t'
                        + String.valueOf(message[4]) + '\t' + '\t' + String.valueOf(message[5]) + '\t' + '\t' + String.valueOf(message[6]) + '\t' + '\t' + String.valueOf(message[7]) + '\t' + '\t'
                        + String.valueOf(message[8]) + '\t' + '\t' + String.valueOf(message[9]) + '\t' + '\t' + String.valueOf(message[10]) + '\t' + '\t' + String.valueOf(message[11]) + '\t' + '\t'
                        + String.valueOf(message[12]) + '\t' + '\t' + String.valueOf(message[13]) + '\t' + '\t' + String.valueOf(message[14]) + '\t' + '\t' + String.valueOf(message[15]));


                lastTimeForFile = sim.getSimulationTimeMillis();

                // if file doesn't exists, then create it
                if (!serialFileRCV.exists()) {
                    serialFileRCV.createNewFile();
                } else if (SerialRCVWrittenForFirst) {
                    serialFileRCV.delete();
                    serialFileRCV.createNewFile();
                    SerialRCVWrittenForFirst = false;
                }

                FileWriter fw = new FileWriter(serialFileRCV.getAbsoluteFile(), true);
                BufferedWriter bw = new BufferedWriter(fw);
                PrintWriter pn = new PrintWriter(fw);
                pn.println(content);
                pn.close();
                logger.info("Done");
            } catch (Exception e) {//Catch exception if any
                logger.info("Error in print dacFile");
            }
        }
        //End of writing

    }

    byte[] serialRequestMsgCreator(int simulationTime, int motePinId) {
        int uValue = 0;
        byte[] emptySerialData = new byte[16];
        for (byte b : emptySerialData) {
            b = 0;
        }
//        logger.info("#simulationTime" + simulationTime);
//        logger.info("#motePinId" + motePinId);

        byte[] simulationTimeInByte = int2Bytes(simulationTime);
        byte[] motePinIdInByte = int2Bytes(motePinId);
        byte[] uValueInByte = int2Bytes(uValue);

        byte[] message = combine(reverseArray(simulationTimeInByte), reverseArray(motePinIdInByte), reverseArray(uValueInByte), emptySerialData);

//        for (byte b : message) {
//            logger.info("#b" + b);
//        }

        return (message);
    }

    byte[] serialMessageCreator(int simulationTime, int motePinId, byte[] serialMessage) {

        int uValue = 0;
        byte[] simulationTimeInByte = int2Bytes(simulationTime);
        byte[] motePinIdInByte = int2Bytes(motePinId);
        byte[] uValueInByte = int2Bytes(uValue);
        byte[] message = combine(reverseArray(simulationTimeInByte), reverseArray(motePinIdInByte), reverseArray(uValueInByte), serialMessage);
        return (message);
    }

    int serialMsgSender(byte[] message, DatagramSocket dgSocket) {

        try {
            InetAddress IPAddress = InetAddress.getByName("localhost");
            DatagramPacket dgPacket = new DatagramPacket(message, message.length, IPAddress, SIMULINK_PORT);
            dgSocket.send(dgPacket);
        } catch (IOException ex) {
            System.out.println("Error in sending dgPacket in adcRequestSender!!");
        }
        byte[] receivedValue = new byte[4];
        DatagramPacket receivedPacket = new DatagramPacket(receivedValue, receivedValue.length);
        try {
            dgSocket.receive(receivedPacket);
        } catch (IOException ex) {
            System.out.println("Error in receive UDP data adcRequestSender!!");
            System.out.println(ex.getMessage());
        }
        byte[] data = receivedPacket.getData();
        return (bytes2Int(data));
    }

    void serialRequestSender(byte[] requestMessage, DatagramSocket dgSocket) {

        try {
            InetAddress IPAddress = InetAddress.getByName("localhost");
            DatagramPacket dgPacket = new DatagramPacket(requestMessage, requestMessage.length, IPAddress, SIMULINK_PORT);
            dgSocket.send(dgPacket);
        } catch (IOException ex) {
            System.out.println("Error in sending dgPacket in adcRequestSender!!");
        }
        byte[] receivedSerialData = new byte[30];
        DatagramPacket receivedPacket = new DatagramPacket(receivedSerialData, receivedSerialData.length);
        try {
            dgSocket.receive(receivedPacket);
        } catch (IOException ex) {
            System.out.println("Error in receive UDP data adcRequestSender!!");
            System.out.println(ex.getMessage());
        }
        byte[] data = reverseArray(receivedPacket.getData());
        for (int i = 0; i < data.length; i++) {
            serialPort.writeByte(data[i]);

        }
    }

    byte[] combine(byte[] one, byte[] two) {
        byte[] combined = new byte[one.length + two.length];
        System.arraycopy(one, 0, combined, 0, one.length);
        System.arraycopy(two, 0, combined, one.length, two.length);
        return combined;
    }

    byte[] combine(byte[] one, byte[] two, byte[] three) {
        byte[] combined = new byte[one.length + two.length + three.length];
        System.arraycopy(one, 0, combined, 0, one.length);
        System.arraycopy(two, 0, combined, one.length, two.length);
        System.arraycopy(three, 0, combined, one.length + two.length, three.length);
        return combined;
    }

    byte[] combine(byte[] one, byte[] two, byte[] three, byte[] four) {
        byte[] combined = new byte[one.length + two.length + three.length + four.length];
        System.arraycopy(one, 0, combined, 0, one.length);
        System.arraycopy(two, 0, combined, one.length, two.length);
        System.arraycopy(three, 0, combined, one.length + two.length, three.length);
        System.arraycopy(four, 0, combined, one.length + two.length + three.length, four.length);
        return combined;
    }

    byte[] reverseArray(byte[] inputArray) {
        byte[] reverseArray = new byte[inputArray.length];
        for (int i = 0; i < inputArray.length; i++) {
            reverseArray[i] = inputArray[(inputArray.length - 1) - i];
        }
        return reverseArray;

    }

    byte[] int2Bytes(int inputInt) {
        byte[] resultByts = new byte[]{
            (byte) ((inputInt >> 24) & 0xff),
            (byte) ((inputInt >> 16) & 0xff),
            (byte) ((inputInt >> 8) & 0xff),
            (byte) ((inputInt >> 0) & 0xff),};
        return resultByts;
    }

    int bytes2Int(byte[] bytes) {
        return ((int) (0xff & bytes[0]) << 24
                | (int) (0xff & bytes[1]) << 16
                | (int) (0xff & bytes[2]) << 8
                | (int) (0xff & bytes[3]) << 0);
    }

    public void releaseInterfaceVisualizer(JPanel panel) {
    }

    public Collection<Element> getConfigXML() {
        return null;
    }

    public boolean setConfigXML(Collection<Element> configXML, boolean visAvailable) {
        return true;
    }

    public void closePlugin() {
        serialPort.deleteSerialDataObserver(serialDataObserver);
        dSocket.close();
    }

    public Mote getMote() {
        return mote;
    }
}
