"""离线协议回归；不打开实际串口。"""
import struct
import unittest
import core_y100m_mqtt_test as dtu


class DtuTransportTests(unittest.TestCase):
    def test_crc_standard_vector(self):
        self.assertEqual(dtu.crc16_modbus(b"123456789"), 0x4B37)

    def test_payload_and_sequence(self):
        frame_class = dtu.load_frame_class()
        payload = dtu.gps_payload(frame_class, 123456, 42)
        for seq in (0, 255, 256):
            wire = dtu.build_wire_frame(payload, seq)
            self.assertEqual(wire[:2], bytes.fromhex("AA55"))
            self.assertEqual(struct.unpack_from("<H", wire, 2)[0], len(payload))
            self.assertEqual(wire[4], seq & 255)
            self.assertEqual(len(wire), len(payload) + 7)
            self.assertEqual(wire[5:-2], payload)
            # Including the appended little-endian CRC gives zero residue.
            self.assertEqual(dtu.crc16_modbus(wire[2:]), 0)
            decoded = frame_class.FromString(wire[5:-2])
            self.assertEqual(decoded.timestamp_ms, 123456)
            self.assertEqual(decoded.header.timestamp_ms, 123456)
            self.assertFalse(decoded.HasField("fast_telemetry"))
            self.assertFalse(decoded.HasField("vehicle_state"))
            self.assertEqual(decoded.motion.gps_speed_kmh, 42)
            self.assertEqual(decoded.ready_to_drive, 1)
            self.assertEqual(decoded.frame_id, 0)
            self.assertEqual(decoded.header.source_id, 0)
            self.assertEqual(decoded.header.seq, 0)

    def test_boundaries(self):
        payload = bytes(range(256)) * 24 + bytes(72)
        self.assertEqual(len(payload), dtu.MAX_PAYLOAD)
        wire = dtu.build_wire_frame(payload, 255)
        self.assertEqual(wire[5:-2], payload)
        self.assertEqual(dtu.crc16_modbus(wire[2:]), 0)
        for invalid in (b"", payload + b"x"):
            with self.assertRaises(ValueError):
                dtu.build_wire_frame(invalid, 0)


if __name__ == "__main__":
    unittest.main()
