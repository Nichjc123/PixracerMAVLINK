using System;
using System.IO.Ports;
using static MAVLink;

namespace MAVLinkFuzzing
{
    class Program
    {
        static void Main(string[] args)
        {
            string portName = "/dev/cu.usbserial-AH01AZMQ"; // UPDATE TO ACTUAL PORT NAME
            int baudRate = 115200;

            var serial = new SerialPort(portName, baudRate, Parity.None, 8, StopBits.One);
            serial.Open();
            Console.WriteLine($"Opened {portName} @ {baudRate} baud");

            MAVLink.MavlinkParse parser = new MAVLink.MavlinkParse();

            // Send MOTOR TEST: motor #1, percent throttle, 30%, for 1 second
            byte targetSystem    = 1;   // Pixracer’s system ID
            byte targetComponent = 1;   // autopilot component ID
            byte gcsSystemId     = 255; // GCS system ID
            byte gcsComponentId  = 190; // GCS component ID
            byte seq             = 0;   // packet sequence

            var motorTest = new mavlink_command_long_t
            {
                target_system    = targetSystem,
                target_component = targetComponent,
                command          = (ushort)MAV_CMD.DO_MOTOR_TEST,
                confirmation     = 0,
                param1           = 1,    // motor instance
                param2           = 0,    // throttle type (0 is percent)
                param3           = 30,   // throttle = 30%
                param4           = 1,    // duration = 1 second
                param5           = 0,
                param6           = 0,
                param7           = 0
            };

            // pack it into a COMMAND_LONG MAVLink packet
            var packet = parser.GenerateMAVLinkPacket20(
                MAVLINK_MSG_ID.COMMAND_LONG,
                motorTest,
                false,
                gcsSystemId,
                gcsComponentId,
                seq++
            );
            serial.Write(packet, 0, packet.Length);
            Console.WriteLine("Sent DO_MOTOR_TEST to spin motor #1 at 30% for 1 s");

            // give it a moment to run
            Thread.Sleep(1100);

            /*
                Read 10 packets from controller
            */
            for (int i = 0; i < 10; i++)
            {
                var msg = parser.ReadPacket(serial.BaseStream);
                if (msg != null && msg.msgid != 0)
                {
                    Console.WriteLine(
                        $"MsgID={(MAVLink.MAVLINK_MSG_ID)msg.msgid}, " +
                        $"SYS={msg.sysid}, COMP={msg.compid}"
                    );
                }
            }
            
            Console.WriteLine("Finished reading messages.");
            serial.Close();
        }
    }
}
