//Pick one of these
//#define USE_OpenNetCF
#define USE_Psimax

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Reflection;
using System.Threading;
using Microsoft.Win32;
using System.Runtime.InteropServices;

using UnityEngine;
using KSP.IO;
using KSP.UI.Screens;

#if (USE_OpenNetCF)
using OpenNETCF.IO.Ports;

namespace KSPSerialIO
{
    [KSPAddon(KSPAddon.Startup.Flight, false)]
    public class KSPSerialPort : MonoBehaviour
    {
        public static SerialPort Port;
        public static string PortNumber;
        public static Boolean DisplayFound = false;
        public static Boolean ControlReceived = false;

        public static VesselData VData;
        public static ControlPacket CPacket;
        private static HandShakePacket HPacket;

        public static VesselControls VControls = new VesselControls();
        public static VesselControls VControlsOld = new VesselControls();

        private static byte[] buffer = new byte[255];
        private static byte rx_len;
        private static byte rx_array_inx;
        private static int structSize;
        private static byte id = 255;

        private const byte HSPid = 0, VDid = 1, Cid = 101; //hard coded values for packet IDS


        public static void sendPacket(object anything)
        {
            byte[] Payload = StructureToByteArray(anything);
            byte header1 = 0xBE;
            byte header2 = 0xEF;
            byte size = (byte)Payload.Length;
            byte checksum = size;

            byte[] Packet = new byte[size + 4];

            //Packet = [header][size][payload][checksum];
            //Header = [Header1=0xBE][Header2=0xEF]
            //size = [payload.length (0-255)]

            for (int i = 0; i < size; i++)
            {
                checksum ^= Payload[i];
            }

            Payload.CopyTo(Packet, 3);
            Packet[0] = header1;
            Packet[1] = header2;
            Packet[2] = size;
            Packet[Packet.Length - 1] = checksum;

            Port.Write(Packet, 0, Packet.Length);
        }

        private void Begin()
        {
            Port = new SerialPort(PortNumber, SettingsNStuff.BaudRate, Parity.None, 8, StopBits.One);
            Port.ReceivedBytesThreshold = 3;
            Port.ReceivedEvent += Port_ReceivedEvent;
        }

        //these are copied from the intarwebs, converts struct to byte array
        private static byte[] StructureToByteArray(object obj)
        {
            int len = Marshal.SizeOf(obj);
            byte[] arr = new byte[len];
            IntPtr ptr = Marshal.AllocHGlobal(len);
            Marshal.StructureToPtr(obj, ptr, true);
            Marshal.Copy(ptr, arr, 0, len);
            Marshal.FreeHGlobal(ptr);
            return arr;
        }

        private static object ByteArrayToStructure(byte[] bytearray, object obj)
        {
            int len = Marshal.SizeOf(obj);

            IntPtr i = Marshal.AllocHGlobal(len);

            Marshal.Copy(bytearray, 0, i, len);

            obj = Marshal.PtrToStructure(i, obj.GetType());

            Marshal.FreeHGlobal(i);

            return obj;
        }
        /*
          private static T ReadUsingMarshalUnsafe<T>(byte[] data) where T : struct
          {
          unsafe
          {
          fixed (byte* p = &data[0])
          {
          return (T)Marshal.PtrToStructure(new IntPtr(p), typeof(T));
          }
          }
          }
        */
        void initializeDataPackets()
        {
            VData = new VesselData();
            VData.id = VDid;

            HPacket = new HandShakePacket();
            HPacket.id = HSPid;
            HPacket.M1 = 1;
            HPacket.M2 = 2;
            HPacket.M3 = 3;

            CPacket = new ControlPacket();

            VControls.ControlGroup = new Boolean[11];
            VControlsOld.ControlGroup = new Boolean[11];
        }

        void Awake()
        {
            if (DisplayFound)
            {
                Debug.Log("KSPSerialIO: running...");
                Begin();
            }
            else
            {
                Debug.Log("KSPSerialIO: Version 0.18.5");
                Debug.Log("KSPSerialIO: Getting serial ports...");
                Debug.Log("KSPSerialIO: Output packet size: " + Marshal.SizeOf(VData).ToString() + "/255");
                initializeDataPackets();

                try
                {
                    //Use registry hack to get a list of serial ports until we get system.io.ports
                    RegistryKey SerialCOMSKey = Registry.LocalMachine.OpenSubKey(@"HARDWARE\\DEVICEMAP\\SERIALCOMM\\");

                    Begin();

                    //print("KSPSerialIO: receive threshold " + Port.ReceivedBytesThreshold.ToString());

                    if (SerialCOMSKey == null)
                    {
                        Debug.Log("KSPSerialIO: Dude do you even win32 serial port??");
                    }
                    else
                    {
                        String[] realports = SerialCOMSKey.GetValueNames();  // get list of all serial devices
                        String[] names = new string[realports.Length + 1];   // make a new list with 1 extra, we put the default port first
                        realports.CopyTo(names, 1);

                        Debug.Log("KSPSerialIO: Found " + names.Length.ToString() + " serial ports");

                        //look through all found ports for our display
                        int j = 0;

                        foreach (string PortName in names)
                        {
                            if (j == 0)  // try default port first
                            {
                                PortNumber = SettingsNStuff.DefaultPort;
                                Debug.Log("KSPSerialIO: trying default port " + PortNumber);
                            }
                            else
                            {
                                PortNumber = (string)SerialCOMSKey.GetValue(PortName);
                                Debug.Log("KSPSerialIO: trying port " + PortName + " - " + PortNumber);
                            }

                            Port.PortName = PortNumber;

                            j++;

                            if (!Port.IsOpen)
                            {
                                try
                                {
                                    Port.Open();
                                }
                                catch (Exception e)
                                {
                                    Debug.Log("Error opening serial port " + Port.PortName + ": " + e.Message);
                                }

                                //secret handshake
                                if (Port.IsOpen && (SettingsNStuff.HandshakeDisable == 0))
                                {
                                    Thread.Sleep(SettingsNStuff.HandshakeDelay);
                                    //Port.DiscardOutBuffer();
                                    //Port.DiscardInBuffer();

                                    sendPacket(HPacket);

                                    //wait for reply
                                    int k = 0;

                                    while (Port.BytesToRead == 0 && k < 15 && !DisplayFound)
                                    {
                                        Thread.Sleep(100);
                                        k++;
                                    }

                                    Port.Close();
                                    if (DisplayFound)
                                    {
                                        Debug.Log("KSPSerialIO: found KSP Display at " + Port.PortName);
                                        break;
                                    }
                                    else
                                    {
                                        Debug.Log("KSPSerialIO: KSP Display not found");
                                    }
                                }
                                else if (Port.IsOpen && (SettingsNStuff.HandshakeDisable == 1))
                                {
                                    DisplayFound = true;
                                    Debug.Log("KSPSerialIO: Handshake disabled, using " + Port.PortName);
                                    break;
                                }
                            }
                            else
                            {
                                Debug.Log("KSPSerialIO: " + PortNumber + "is already being used.");
                            }
                        }
                    }

                }
                catch (Exception e)
                {
                    print(e.Message);
                }
            }
        }

        private string readline()
        {
            string result = null;
            char c;
            int j = 0;

            c = (char)Port.ReadByte();
            while (c != '\n' && j < 255)
            {
                result += c;
                c = (char)Port.ReadByte();
                j++;
            }
            return result;
        }

        private void Port_ReceivedEvent(object sender, SerialReceivedEventArgs e)
        {
            while (Port.BytesToRead > 0)
            {
                if (processCOM())
                {
                    switch (id)
                    {
                        case HSPid:
                            HPacket = (HandShakePacket)ByteArrayToStructure(buffer, HPacket);
                            Invoke("HandShake", 0);

                            if ((HPacket.M1 == 3) && (HPacket.M2 == 1) && (HPacket.M3 == 4))
                            {
                                DisplayFound = true;

                            }
                            else
                            {
                                DisplayFound = false;
                            }
                            break;
                        case Cid:
                            VesselControls();
                            //Invoke("VesselControls", 0);
                            break;
                        default:
                            Invoke("Unimplemented", 0);
                            break;
                    }
                }
            }
        }

        private static bool processCOM()
        {
            byte calc_CS;

            if (rx_len == 0)
            {
                while (Port.ReadByte() != 0xBE)
                {
                    if (Port.BytesToRead == 0)
                        return false;
                }

                if (Port.ReadByte() == 0xEF)
                {
                    rx_len = (byte)Port.ReadByte();
                    id = (byte)Port.ReadByte();
                    rx_array_inx = 1;

                    switch (id)
                    {
                        case HSPid:
                            structSize = Marshal.SizeOf(HPacket);
                            break;
                        case Cid:
                            structSize = Marshal.SizeOf(CPacket);
                            break;
                    }

                    //make sure the binary structs on both Arduino and plugin are the same size.
                    if (rx_len != structSize || rx_len == 0)
                    {
                        rx_len = 0;
                        return false;
                    }
                }
                else
                {
                    return false;
                }
            }
            else
            {
                while (Port.BytesToRead > 0 && rx_array_inx <= rx_len)
                {
                    buffer[rx_array_inx++] = (byte)Port.ReadByte();
                }
                buffer[0] = id;

                if (rx_len == (rx_array_inx - 1))
                {
                    //seem to have got whole message
                    //last uint8_t is CS
                    calc_CS = rx_len;
                    for (int i = 0; i < rx_len; i++)
                    {
                        calc_CS ^= buffer[i];
                    }

                    if (calc_CS == buffer[rx_array_inx - 1])
                    {//CS good
                        rx_len = 0;
                        rx_array_inx = 1;
                        return true;
                    }
                    else
                    {
                        //failed checksum, need to clear this out anyway
                        rx_len = 0;
                        rx_array_inx = 1;
                        return false;
                    }
                }
            }

            return false;
        }

        private void HandShake()
        {
            Debug.Log("KSPSerialIO: Handshake received - " + HPacket.M1.ToString() + HPacket.M2.ToString() + HPacket.M3.ToString());
        }

        private void VesselControls()
        {
            CPacket = (ControlPacket)ByteArrayToStructure(buffer, CPacket);

            VControls.SAS = BitMathByte(CPacket.MainControls, 7);
            VControls.RCS = BitMathByte(CPacket.MainControls, 6);
            VControls.Lights = BitMathByte(CPacket.MainControls, 5);
            VControls.Gear = BitMathByte(CPacket.MainControls, 4);
            VControls.Brakes = BitMathByte(CPacket.MainControls, 3);
            VControls.Precision = BitMathByte(CPacket.MainControls, 2);
            VControls.Abort = BitMathByte(CPacket.MainControls, 1);
            VControls.Stage = BitMathByte(CPacket.MainControls, 0);
            VControls.Pitch = (float)CPacket.Pitch / 1000.0F;
            VControls.Roll = (float)CPacket.Roll / 1000.0F;
            VControls.Yaw = (float)CPacket.Yaw / 1000.0F;
            VControls.TX = (float)CPacket.TX / 1000.0F;
            VControls.TY = (float)CPacket.TY / 1000.0F;
            VControls.TZ = (float)CPacket.TZ / 1000.0F;
            VControls.WheelSteer = (float)CPacket.WheelSteer / 1000.0F;
            VControls.Throttle = (float)CPacket.Throttle / 1000.0F;
            VControls.WheelThrottle = (float)CPacket.WheelThrottle / 1000.0F;
            VControls.SASMode = (int)CPacket.SASMode;

            for (int j = 1; j <= 10; j++)
            {
                VControls.ControlGroup[j] = BitMathUshort(CPacket.ControlGroup, j);
            }

            ControlReceived = true;
            //Debug.Log("KSPSerialIO: ControlPacket received");
        }

        private Boolean BitMathByte(byte x, int n)
        {
            return ((x >> n) & 1) == 1;
        }

        private Boolean BitMathUshort(ushort x, int n)
        {
            return ((x >> n) & 1) == 1;
        }

        private void Unimplemented()
        {
            Debug.Log("KSPSerialIO: Packet id unimplemented");
        }

        private static void debug()
        {
            Debug.Log(Port.BytesToRead.ToString() + "BTR");
        }


        public static void ControlStatus(int n, bool s)
        {
            if (s)
                VData.ActionGroups |= (UInt16)(1 << n);       // forces nth bit of x to be 1.  all other bits left alone.
            else
                VData.ActionGroups &= (UInt16)~(1 << n);      // forces nth bit of x to be 0.  all other bits left alone.
        }

        void OnDestroy()
        {
            if (KSPSerialPort.Port.IsOpen)
            {
                KSPSerialPort.Port.Close();
                Port.ReceivedEvent -= Port_ReceivedEvent;
                Debug.Log("KSPSerialIO: Port closed");
            }
        }
    }
}
#endif //(COMMS == OpenNetCF)

#if (USE_Psimax)
using Psimax.IO.Ports;

namespace KSPSerialIO
{
    [KSPAddon(KSPAddon.Startup.Flight, false)]
    public class KSPSerialPort : MonoBehaviour
    {
        public static Psimax.IO.Ports.SerialPort Port;
        public static string PortNumber;
        public static Boolean DisplayFound = false;
        public static Boolean ControlReceived = false;

        public static VesselData VData;
        public static ControlPacket CPacket;
        private static HandShakePacket HPacket;

        public static VesselControls VControls = new VesselControls();
        public static VesselControls VControlsOld = new VesselControls();

        private const int MaxPayloadSize = 255;
        enum ReceiveStates: byte {
            FIRSTHEADER, // Waiting for first header
            SECONDHEADER, // Waiting for second header
            SIZE, // Waiting for payload size
            PAYLOAD, // Waiting for rest of payload
            CS // Waiting for checksum
        }
        private static ReceiveStates CurrentState = ReceiveStates.FIRSTHEADER;
        private static byte CurrentPacketLength;
        private static byte CurrentBytesRead;
        // Guards access to data shared between threads
        private static Mutex SerialMutex = new Mutex();
        // Serial worker uses this buffer to read bytes
        private static byte[] PayloadBuffer = new byte[MaxPayloadSize];
        // Buffer for sharing packets from serial worker to main thrad
        private static volatile bool NewPacketFlag = false;
        private static volatile byte[] NewPacketBuffer = new byte[MaxPayloadSize];
        // Semaphore to indicate whether the serial worker should do work
        private static volatile bool doSerialRead = true;
        private static Thread SerialThread;

        private const byte HSPid = 0, VDid = 1, Cid = 101; //hard coded values for packet IDS

        public static void InboundPacketHandler()
        {
            SerialMutex.WaitOne();
            NewPacketFlag = false;
            switch (NewPacketBuffer[0])
            {
                case HSPid:
                    HPacket = (HandShakePacket)ByteArrayToStructure(NewPacketBuffer, HPacket);
                    SerialMutex.ReleaseMutex();
                    HandShake();
                    if ((HPacket.M1 == 3) && (HPacket.M2 == 1) && (HPacket.M3 == 4)) {
                        DisplayFound = true;
                    } else
                    {
                        DisplayFound = false;
                    }
                    break;
                case Cid:
                    CPacket = (ControlPacket)ByteArrayToStructure(PayloadBuffer, CPacket);
                    SerialMutex.ReleaseMutex();
                    VesselControls();
                    break;
                default:
                    SerialMutex.ReleaseMutex();
                    Debug.Log("KSPSerialIO: Packet id unimplementd");
                    break;
            }

        }

        public static void sendPacket(object anything)
        {
            byte[] Payload = StructureToByteArray(anything);
            byte header1 = 0xBE;
            byte header2 = 0xEF;
            byte size = (byte)Payload.Length;
            byte checksum = size;

            byte[] Packet = new byte[size + 4];

            //Packet = [header][size][payload][checksum];
            //Header = [Header1=0xBE][Header2=0xEF]
            //size = [payload.length (0-255)]

            for (int i = 0; i < size; i++)
            {
                checksum ^= Payload[i];
            }

            Payload.CopyTo(Packet, 3);
            Packet[0] = header1;
            Packet[1] = header2;
            Packet[2] = size;
            Packet[Packet.Length - 1] = checksum;

            Port.Write(Packet, 0, Packet.Length);
        }

        private void Begin()
        {
            Port = new SerialPort(PortNumber, SettingsNStuff.BaudRate, Parity.None, 8, StopBits.One);
            SerialThread = new Thread(SerialWorker);
            SerialThread.Start();
            while (!SerialThread.IsAlive);
        }

        private void Update()
        {
            if (NewPacketFlag)
            {
                InboundPacketHandler();
            }
        }

        private void SerialWorker()
        {
            byte[] buffer = new byte[MaxPayloadSize + 4];
            Action SerialRead = null;
            Debug.Log("KSPSerialIO: Serial Worker thread started");
            SerialRead = delegate {
                try
                {
                    Port.BaseStream.BeginRead(buffer, 0, buffer.Length, delegate (IAsyncResult ar) {
                            try
                            {
                                int actualLength = Port.BaseStream.EndRead(ar);
                                byte[] received = new byte[actualLength];
                                Buffer.BlockCopy(buffer, 0, received, 0, actualLength);
                                ReceivedDataEvent(received, actualLength);
                            }
                            catch (IOException exc)
                            {
                                Debug.Log("IOException in SerialWorker :(");
                                Debug.Log(exc.ToString());
                            }
                        }, null);
                }
                catch (InvalidOperationException)
                {
                    Debug.Log("KSPSerialIO: Trying to read port that isn't open. Sleeping");
                    Thread.Sleep(500);
                }
            };

            doSerialRead = true;
            while (doSerialRead)
            {
                SerialRead();
            }
            Debug.Log("KSPSerialIO: Serial worker thread shutting down.");
        }

        private void ReceivedDataEvent(byte[] ReadBuffer, int BufferLength)
        {
            for (int x=0; x<BufferLength; x++)
            {
                switch(CurrentState)
                {
                    case ReceiveStates.FIRSTHEADER:
                        if (ReadBuffer[x] == 0xBE)
                        {
                            CurrentState = ReceiveStates.SECONDHEADER;
                        }
                        break;
                    case ReceiveStates.SECONDHEADER:
                        if (ReadBuffer[x] == 0xEF)
                        {
                            CurrentState = ReceiveStates.SIZE;
                        } else
                        {
                            CurrentState = ReceiveStates.FIRSTHEADER;
                        }
                        break;
                    case ReceiveStates.SIZE:
                        CurrentPacketLength = ReadBuffer[x];
                        CurrentBytesRead = 0;
                        CurrentState = ReceiveStates.PAYLOAD;
                        break;
                    case ReceiveStates.PAYLOAD:
                        PayloadBuffer[CurrentBytesRead] = ReadBuffer[x];
                        CurrentBytesRead++;
                        if (CurrentBytesRead == CurrentPacketLength)
                        {
                            CurrentState = ReceiveStates.CS;
                        }
                        break;
                    case ReceiveStates.CS:
                        if (CompareChecksum(ReadBuffer[x]))
                        {
                            SerialMutex.WaitOne();
                            Buffer.BlockCopy(PayloadBuffer, 0, NewPacketBuffer, 0, CurrentBytesRead);
                            NewPacketFlag = true;
                            SerialMutex.ReleaseMutex();
                            // Seedy hack: Handshake happens during scene
                            // load before Update() is ever called
                            if (!DisplayFound)
                            {
                                InboundPacketHandler();
                            }
                        }
                        CurrentState = ReceiveStates.FIRSTHEADER;
                        break;
                }
            }
        }

        private static Boolean CompareChecksum(byte readCS)
        {
            byte calcCS = CurrentPacketLength;
            for (int i=0; i<CurrentPacketLength; i++)
            {
                calcCS ^= PayloadBuffer[i];
            }
            return (calcCS == readCS);
        }

        //these are copied from the intarwebs, converts struct to byte array
        private static byte[] StructureToByteArray(object obj)
        {
            int len = Marshal.SizeOf(obj);
            byte[] arr = new byte[len];
            IntPtr ptr = Marshal.AllocHGlobal(len);
            Marshal.StructureToPtr(obj, ptr, true);
            Marshal.Copy(ptr, arr, 0, len);
            Marshal.FreeHGlobal(ptr);
            return arr;
        }

        private static object ByteArrayToStructure(byte[] bytearray, object obj)
        {
            int len = Marshal.SizeOf(obj);

            IntPtr i = Marshal.AllocHGlobal(len);

            Marshal.Copy(bytearray, 0, i, len);

            obj = Marshal.PtrToStructure(i, obj.GetType());

            Marshal.FreeHGlobal(i);

            return obj;
        }
        /*
          private static T ReadUsingMarshalUnsafe<T>(byte[] data) where T : struct
          {
          unsafe
          {
          fixed (byte* p = &data[0])
          {
          return (T)Marshal.PtrToStructure(new IntPtr(p), typeof(T));
          }
          }
          }
        */
        void initializeDataPackets()
        {
            VData = new VesselData();
            VData.id = VDid;

            HPacket = new HandShakePacket();
            HPacket.id = HSPid;
            HPacket.M1 = 1;
            HPacket.M2 = 2;
            HPacket.M3 = 3;

            CPacket = new ControlPacket();

            VControls.ControlGroup = new Boolean[11];
            VControlsOld.ControlGroup = new Boolean[11];
        }

        void Awake()
        {
            if (DisplayFound)
            {
                Debug.Log("KSPSerialIO: running...");
                Begin();
            }
            else
            {
                String version = System.Reflection.Assembly.GetExecutingAssembly()
                    .GetName().Version.ToString();
                Debug.Log(String.Format("KSPSerialIO: Version {0}", version));
                Debug.Log("KSPSerialIO: Getting serial ports...");
                Debug.Log(String.Format("KSPSerialIO: Output packet size: {0}/{1}",
                                        Marshal.SizeOf(VData).ToString(),
                                        MaxPayloadSize));
                initializeDataPackets();

                try
                {
                    //Use registry hack to get a list of serial ports until we get system.io.ports
                    RegistryKey SerialCOMSKey = Registry.LocalMachine.OpenSubKey(@"HARDWARE\\DEVICEMAP\\SERIALCOMM\\");

                    Begin();

                    //print("KSPSerialIO: receive threshold " + Port.ReceivedBytesThreshold.ToString());

                    String[] PortNames;
                    if (SerialCOMSKey == null)
                    {
                        Debug.Log("KSPSerialIO: Dude do you even win32 serial port??");
                        PortNames = new String[1];
                        PortNames[0] = SettingsNStuff.DefaultPort;
                    }
                    else
                    {
                        String[] realports = SerialCOMSKey.GetValueNames();  // get list of all serial devices
                        PortNames = new String[realports.Length + 1];   // make a new list with 1 extra, we put the default port first
                        realports.CopyTo(PortNames, 1);
                    }

                    Debug.Log(String.Format("KSPSerialIO: Found {0} serial ports",
                                            PortNames.Length));

                    //look through all found ports for our display
                    for (int j=0; j < PortNames.Length; j++)
                    {
                        if (j == 0)  // try default port first
                        {
                            PortNumber = SettingsNStuff.DefaultPort;
                            Debug.Log("KSPSerialIO_mm: trying default port " + PortNumber);
                        }
                        else
                        {
                            PortNumber = (string)SerialCOMSKey.GetValue(PortNames[j]);
                            Debug.Log("KSPSerialIO: trying port " + PortNames[j] + " - " + PortNumber);
                        }
                        Port.PortName = PortNumber;

                        if (!Port.IsOpen)
                        {
                            Debug.Log("KSPSerialIO_mm: Opening " + PortNames[j]);
                            try
                            {
                                Port.Open();
                            }
                            catch (IOException e){
                                Debug.Log("I/O Error opening serial port " + Port.PortName + ": " + e.Message);
                            }
                            catch (Exception e)
                            {
                                Debug.Log("Error opening serial port " + Port.PortName + ": " + e.Message);
                            }

                            //secret handshake
                            if (Port.IsOpen && (SettingsNStuff.HandshakeDisable == 0))
                            {
                                Debug.Log("KSPSerialIO_mm: Port " + Port.PortName + " is trying handshake.");
                                Thread.Sleep(SettingsNStuff.HandshakeDelay);
                                //Port.DiscardOutBuffer();
                                //Port.DiscardInBuffer();
                                
                                sendPacket(HPacket);

                                //wait for reply
                                Debug.Log("KSPSerialIO_mm: Waiting for reply.");
                                int k = 0;
                                while (k < 15 && !DisplayFound)
                                {
                                    Thread.Sleep(100);
                                    k++;
                                }

                                Port.Close();
                                if (DisplayFound)
                                {
                                    Debug.Log("KSPSerialIO: found KSP Display at " + Port.PortName);
                                    break;
                                }
                                else
                                {
                                    Debug.Log("KSPSerialIO: KSP Display not found");
                                }
                            }
                            else if (Port.IsOpen && (SettingsNStuff.HandshakeDisable == 1))
                            {
                                DisplayFound = true;                                    
                                Debug.Log("KSPSerialIO: Handshake disabled, using " + Port.PortName);
                                break;
                            }
                            else{
                                Debug.Log("KSPSerialIO_mm: " + PortNames[j] + " is not open.");
                            }
                        }
                        else
                        {
                            Debug.Log("KSPSerialIO: " + PortNumber + "is already being used.");
                        }
                    }
                }
                catch (Exception e)
                {
                    print(e.Message);
                }
            }
        }

        private static void HandShake()
        {
            Debug.Log("KSPSerialIO: Handshake received - " + HPacket.M1.ToString() + HPacket.M2.ToString() + HPacket.M3.ToString());
        }

        private static void VesselControls()
        {

            VControls.SAS = BitMathByte(CPacket.MainControls, 7);
            VControls.RCS = BitMathByte(CPacket.MainControls, 6);
            VControls.Lights = BitMathByte(CPacket.MainControls, 5);
            VControls.Gear = BitMathByte(CPacket.MainControls, 4);
            VControls.Brakes = BitMathByte(CPacket.MainControls, 3);
            VControls.Precision = BitMathByte(CPacket.MainControls, 2);
            VControls.Abort = BitMathByte(CPacket.MainControls, 1);
            VControls.Stage = BitMathByte(CPacket.MainControls, 0);
            VControls.Pitch = (float)CPacket.Pitch / 1000.0F;
            VControls.Roll = (float)CPacket.Roll / 1000.0F;
            VControls.Yaw = (float)CPacket.Yaw / 1000.0F;
            VControls.TX = (float)CPacket.TX / 1000.0F;
            VControls.TY = (float)CPacket.TY / 1000.0F;
            VControls.TZ = (float)CPacket.TZ / 1000.0F;
            VControls.WheelSteer = (float)CPacket.WheelSteer / 1000.0F;
            VControls.Throttle = (float)CPacket.Throttle / 1000.0F;
            VControls.WheelThrottle = (float)CPacket.WheelThrottle / 1000.0F;

            for (int j = 1; j <= 10; j++)
            {
                VControls.ControlGroup[j] = BitMathUshort(CPacket.ControlGroup, j);
            }

            ControlReceived = true;
            //Debug.Log("KSPSerialIO: ControlPacket received");
        }

        private static Boolean BitMathByte(byte x, int n)
        {
            return ((x >> n) & 1) == 1;
        }

        private static Boolean BitMathUshort(ushort x, int n)
        {
            return ((x >> n) & 1) == 1;
        }

        private static void Unimplemented()
        {
            Debug.Log("KSPSerialIO: Packet id unimplemented");
        }

        private static void debug()
        {
            Debug.Log(Port.BytesToRead.ToString() + "BTR");
        }


        public static void ControlStatus(int n, Boolean s)
        {
            if (s)
                VData.ActionGroups |= (UInt16)(1 << n);       // forces nth bit of x to be 1.  all other bits left alone.
            else
                VData.ActionGroups &= (UInt16)~(1 << n);      // forces nth bit of x to be 0.  all other bits left alone.
        }

        public static void PortCleanup()
        {
            if (KSPSerialPort.Port.IsOpen)
            {
                doSerialRead = false;
                KSPSerialPort.Port.Close();
                KSPSerialPort.Port.Dispose();
                Debug.Log("KSPSerialIO: Port closed");
            }
        }
    }
}
#endif //(USE_Psimax)

