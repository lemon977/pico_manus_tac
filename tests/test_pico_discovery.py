import socket
import struct
import unittest
from unittest import mock

import pico_discovery


class DiscoveryFrameTests(unittest.TestCase):
    def test_frame_matches_xrobotoolkit_wire_format(self):
        frame = pico_discovery.pack_discovery_frame(
            "192.168.50.12", timestamp_s=123456789
        )
        head, command, payload_size = struct.unpack("<BBI", frame[:6])
        payload_end = 6 + payload_size
        timestamp, tail = struct.unpack("<QB", frame[payload_end:])

        self.assertEqual(pico_discovery.HEAD_SERVER, head)
        self.assertEqual(pico_discovery.BCAST_CMD_TCPIP, command)
        self.assertEqual(b"192.168.50.12", frame[6:payload_end])
        self.assertEqual(123456789, timestamp)
        self.assertEqual(pico_discovery.TAIL, tail)

    def test_ipconfig_parser_works_for_chinese_and_english_output(self):
        output = """
           IPv4 地址 . . . . . . . . . . . . : 192.168.10.8(首选)
           IPv4 Address. . . . . . . . . . . : 10.77.0.2(Preferred)
           Subnet Mask . . . . . . . . . . . : 255.255.255.0
           IPv4 Address. . . . . . . . . . . : 127.0.0.1
        """
        self.assertEqual(
            {"192.168.10.8", "10.77.0.2"},
            pico_discovery.parse_ipconfig_ipv4s(output),
        )

    def test_discovery_merges_all_address_sources(self):
        with mock.patch.object(
            pico_discovery, "_hostname_ipv4s", return_value={"192.168.1.9"}
        ), mock.patch.object(
            pico_discovery, "_route_probe_ipv4s", return_value={"10.0.0.5"}
        ), mock.patch.object(
            pico_discovery, "_windows_ipconfig_ipv4s",
            return_value={"192.168.1.9", "172.20.10.2"},
        ):
            result = pico_discovery.discover_local_ipv4s()
        self.assertEqual(["10.0.0.5", "172.20.10.2", "192.168.1.9"], result)


class FakeBroadcastSocket:
    def __init__(self):
        self.options = []
        self.bound = None
        self.packets = []
        self.closed = False

    def setsockopt(self, level, option, value):
        self.options.append((level, option, value))

    def bind(self, address):
        self.bound = address

    def sendto(self, payload, address):
        self.packets.append((payload, address))
        return len(payload)

    def close(self):
        self.closed = True


class BroadcastCycleTests(unittest.TestCase):
    def test_each_ip_is_bound_to_its_own_interface(self):
        created = []

        def factory(*_args):
            instance = FakeBroadcastSocket()
            created.append(instance)
            return instance

        sent, errors = pico_discovery.send_broadcast_cycle(
            ["192.168.5.12", "10.10.0.7"], socket_factory=factory
        )

        self.assertEqual(4, sent)
        self.assertEqual(0, errors)
        self.assertEqual(("192.168.5.12", 0), created[0].bound)
        self.assertEqual(("10.10.0.7", 0), created[1].bound)
        self.assertEqual(
            {("192.168.5.255", 29888), ("255.255.255.255", 29888)},
            {address for _payload, address in created[0].packets},
        )
        self.assertTrue(all(instance.closed for instance in created))
        self.assertIn(
            (socket.SOL_SOCKET, socket.SO_BROADCAST, 1), created[0].options
        )


if __name__ == "__main__":
    unittest.main()
