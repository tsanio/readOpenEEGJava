package readingopeneeg;

import gnu.io.CommPort;
import gnu.io.CommPortIdentifier;
import gnu.io.SerialPort;
import java.io.IOException;
import java.io.InputStream;
import java.util.concurrent.TimeUnit;
import javax.management.remote.JMXConnectorFactory;
import javax.swing.SwingWorker;

/**
 * Reading EEG-data from Open EEG device. I used OLIMEX EEG-SMT to try this.
 * 
 * If running this in Windows environment 64Bit JDK might not work and you might have to set JDK to use 32Bit version for the RXTXcommm to work properly.
 * 1. Download Java JDK 32Bit and install it to parallel folder.
 * 2. In Netbeans Tools -> Java Platforms -> Add platform and name it 32Bit version to distinguish it later.
 * Project Properties -> Libraries -> Java Platform: [Choose here the 32Bit java platform ie "JDK 1.8 32bit"]
 * 
 * Add also corresponding path to runtime libraries Project Properties -> Run -> VM-settings: "-Djava.library.path=.\.\.\lib\rxtx-2.1-7-bins-r2\Windows\i368-mingw32"
 * Select from there according your platform.
 * 
 * @author Toni Sanio
 */
public class ReadingOpenEEG {

    // See OpenEEG Firmware http://openeeg.sourceforge.net/doc/modeeg/firmware/modeeg-p2.c
    // 17-byte packets are transmitted from the ModularEEG at 256Hz,
    // using 1 start bit, 8 data bits, 1 stop bit, no parity, 57600 bits per second.
    // Minimial transmission speed is 256Hz * sizeof(modeeg_packet) * 10 = 43520 bps.
    
    // Serial port instance.
    SerialPort serialPort;
    
    // Input stream.
    InputStream in;
    
    // Reading thread.
    Thread readThread;
    
    // Flag to keep read thread running.
    boolean runReadingThread=false;        
    
    // Just default buffer size for data.
    int bufferSize = 1024*4;
    
    // Sample rate which determines when sample is ready.
    int sampleRate = 256;
    
    // Default settings for serial.
    int baudRate = 57600;
    int dataBits = SerialPort.DATABITS_8;
    int stopBits = SerialPort.STOPBITS_1;
    int parity   = SerialPort.PARITY_NONE;    
        
    // Chosen port name.
    private String port = "COM2";  //"/dev/ttyUSB0", // Linux
    
    // How many sensors we read from the device.
    int sensorAmount = 2;
    
    // Hellpers to count packets.
    int packetCount = 0;        
    long sampleStartTimeNs = System.nanoTime();
    long sampleDurationNs = 0;
    
    // Start time of the application.
    static long startTimeMs;            
    
    // How long to run the application before quit.
    static long runAppTimeMs=10000;
    
    // Raw buffer for sample.
    double[][] rawBuffer = new double[sensorAmount][sampleRate];
    
    /**
     * Application reads eeg from com port for awhile.
     */
    public static void main(String[] args) {                
        
        ReadingOpenEEG readingOpenEEG = new ReadingOpenEEG();        
        if (readingOpenEEG.connect()) {
            
            startTimeMs=System.currentTimeMillis();
            // Run short period of time to see what is ccming from the port and exit..
            while (System.currentTimeMillis() - startTimeMs < runAppTimeMs) {
                // ..
            }
            readingOpenEEG.disconnect();
            
        }
        
    }
    
    // Receive data from COMport.
    public void receiveData(byte[] buffer, int len) {
        
        // Checking success of the reading from serial.
        if (len>0) {                    
            
            // Loop through all data.
            for (int i=0;i<len;i++) {                                
                
                // Find the starting point of each packet (90).
                if ((int)(buffer[i])==90 && i+15<=len) {                                                           
                    
                    // Single packet found. Check that it's not broken and it's correct point. 15th index should be 0-15 to indicate status.
                    if ((int)(buffer[i+15]) >= 0 && (int)(buffer[i+15]) <= 15)
                    {                        
                                    
                        packetCount++;
                        addNewRawValueToChannel(0, buffer[i + 3], buffer[i + 4]);
                        addNewRawValueToChannel(1, buffer[i + 5], buffer[i + 6]);
                        
                        // Flush when we have complete sample again.
                        if (packetCount>=sampleRate) {
                        
                            packetCount=0;
                            sampleDurationNs = System.nanoTime() - sampleStartTimeNs;                            
                            sampleStartTimeNs = System.nanoTime();
                        
                            // Sends the 256 or so packets to the listeners. This way they receive always the 256 samples around the 1 sec interval.
                            broadcastSampleDone();                                                        
                                                        
                        }
                        
                    } else {
                        
                        // Some packages are not okay?..                         

                    }
                }

            }            
            
        } // len > 0        
        
    }
    
    // New raw value to channel which describes the sensor.
    public void addNewRawValueToChannel(int sensorIndex, int chanhigh, int chanlow) {    
        
        // Calculate each channel data from lowbit and highbit. It's a 16bit value divided to 8bits :)
        int intValue  = chanhigh * 256 + chanlow; // 256 x chan high bytes + chanlow bytes. Google If you are unsure about it.
        
        // Update the packet on the sample.
        rawBuffer[sensorIndex][packetCount-1] = intValue;        
        
   } 
    
    /** 
     * After 256 or so packets are received we can broadcast sample done and ready to be read for other use.
     */
    public void broadcastSampleDone() {        
        // Read rawBuffer somewhere to get values?
        System.out.println(sampleRate + " packets received in " + getLastSampleDurationMs() + "ms.");
    }
    
    // Just to convert nanoseconds to millis.
    public double getLastSampleDurationMs() {
        return (double)(TimeUnit.NANOSECONDS.toMillis(sampleDurationNs));
    }
    
    /**
     * Just connects to port.
     * @return true / false depending If it succeeded or not.
     */
    public boolean connect() {
        
        try {
            CommPortIdentifier portIdentifier = CommPortIdentifier.getPortIdentifier(port);        
            if ( portIdentifier.isCurrentlyOwned() )
            {
                System.err.println("Port " + port + " is currently in use");
                return false;
            }
            else
            {
                CommPort commPort = portIdentifier.open(this.getClass().getName(), 2000);

                if ( commPort instanceof SerialPort )
                {
                    serialPort = (SerialPort) commPort;
                    serialPort.setSerialPortParams( baudRate,
                                                    dataBits,
                                                    stopBits,
                                                    parity);

                    in = serialPort.getInputStream();                    

                    readThread = new Thread(new SerialReader(in, this));
                    readThread.start();

                } else {
                    System.err.println("Only serial ports are handled currently.");
                    return false;
                }
            }
        } catch (Exception e) {
            System.err.println("Exception while connecting to port " + port + ": " + e.getMessage());
            return false;
        }
        
        System.out.println("Com port connected " + port);
        return true;
        
    }
    
            
    /**
     * Reads incoming data and pass it forward.
     */
    public static class SerialReader implements Runnable 
    {
        InputStream in;                
        ReadingOpenEEG holder;
        
        public SerialReader ( InputStream in, ReadingOpenEEG holder )
        {
            this.holder = holder;
            this.holder.runReadingThread = true;
            this.in = in;                        
        }        
        
        public void run ()
        {                        
            byte[] buffer = new byte[holder.bufferSize];
            int len = -1;
            try
            {
                while ( ( len = this.in.read(buffer)) > -1 && this.holder.runReadingThread)
                {                    
                    if (len>0)                        
                        holder.receiveData(buffer, len);
                }
                System.out.println("Reading thread exit.");
            }
            catch ( IOException e )
            {
                System.err.println(e.getStackTrace());
            }         
        }
    }
    
    
    /**
     * Just method to disconnect and close the reading thread.
     */
    public void disconnect() {        
        
        // Set this false anyway to close the reading thread in case it is still on.
        runReadingThread = false;
        
        if (isConnected()) {
            
            System.out.println("Closing serial port " + port);                                    
            
            if (in!=null) {                
                
                try {
                
                    in.close();                
                    
                } catch(Exception e) {
                    
                    System.err.println("Exception when closing input stream " + e + ".");
                    
                }
                
            } 
            
            serialPort.close();
            
            System.out.println("Serial port " + port + " disconnected.");        
            
            serialPort = null;
            
        
        } else {
            
            System.out.println("No need to close serial port: Not connected.");
            
        }                
        
    } 
    
    /**
     * Just check If we are connected to serial port.
     * @return 
     */
    public boolean isConnected() {
        if (serialPort!=null)
            return true;
        return false;
    }
    
}
