import tempfile
import socket
import struct
import threading
import unittest
from pathlib import Path

import numpy as np

from export_dataset import match_video_frames
from pico_receiver import VideoReceiver


class VstAccessUnitTests(unittest.TestCase):
    def test_config_only_packet_is_not_a_picture(self):
        packet = b"\x00\x00\x00\x01\x67abc\x00\x00\x00\x01\x68def"
        self.assertFalse(VideoReceiver._has_vcl_nal(packet))

    def test_idr_packet_is_a_picture(self):
        packet = b"\x00\x00\x00\x01\x67abc\x00\x00\x01\x65picture"
        self.assertTrue(VideoReceiver._has_vcl_nal(packet))

    def test_session_writer_waits_for_config_and_idr(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "vst.h264"
            receiver = VideoReceiver(0, view=False)
            server, client = socket.socketpair()
            worker = threading.Thread(
                target=receiver.handle_stream, args=(server,), daemon=True)
            receiver.start_recording(str(output))
            worker.start()

            def send(au):
                client.sendall(struct.pack(">I", len(au)) + au)

            # A P frame from the warm stream must not make the session depend
            # on video bytes written before START.
            send(b"\x00\x00\x00\x01\x41old-p-frame")
            send(b"\x00\x00\x00\x01\x67sps\x00\x00\x00\x01\x68pps")
            send(b"\x00\x00\x00\x01\x65idr")
            send(b"\x00\x00\x00\x01\x41new-p-frame")
            self.assertTrue(receiver.wait_recording_ready(1.0))
            client.close()
            worker.join(1.0)
            frames, byte_count = receiver.stop_recording()

            payload = output.read_bytes()
            types = [kind for kind, _ in receiver._annexb_nals(payload)]
            self.assertEqual(types, [7, 8, 5, 1])
            self.assertNotIn(b"old-p-frame", payload)
            self.assertEqual(frames, 2)
            self.assertEqual(byte_count, len(payload))
            self.assertEqual(len((Path(directory) / "vst.ts.jsonl").read_text().splitlines()), 2)
            self.assertEqual(len((Path(directory) / "vst.qpc.ts.jsonl").read_text().splitlines()), 2)


class LegacySidecarTests(unittest.TestCase):
    def test_leading_config_timestamp_is_removed(self):
        with tempfile.TemporaryDirectory() as directory:
            sidecar = Path(directory) / "vst.ts.jsonl"
            sidecar.write_text("100\n100\n120\n140\n", encoding="utf-8")
            result = match_video_frames(
                np.asarray([100, 120, 140], dtype=np.int64),
                sidecar,
                gate_ms=1.0,
                video_frame_count=3,
            )
        np.testing.assert_array_equal(result, np.asarray([0, 1, 2], dtype=np.int32))

    def test_ambiguous_excess_timestamp_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            sidecar = Path(directory) / "vst.ts.jsonl"
            sidecar.write_text("90\n100\n120\n140\n", encoding="utf-8")
            with self.assertRaises(ValueError):
                match_video_frames(
                    np.asarray([100], dtype=np.int64),
                    sidecar,
                    video_frame_count=3,
                )


if __name__ == "__main__":
    unittest.main()
