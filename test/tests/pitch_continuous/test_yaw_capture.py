"""Offline checks. No COM port is opened and no motor command is transmitted."""
import contextlib
from collections import Counter
from collections import deque
import importlib.util
import io
from pathlib import Path
import sys
import threading
import tempfile
import unittest
from unittest.mock import patch

path = Path(sys.argv.pop(1))
spec = importlib.util.spec_from_file_location("yaw_capture_under_test", path)
capture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(capture)


class CaptureTests(unittest.TestCase):
    def test_endpoint_acceptance_and_bad_data(self):
        rows = [(i*.02, (0, 0, 0, .4, 0, 0, 20, 0, 0)) for i in range(26)]
        result = capture.evaluate_leg(rows, .5)
        self.assertTrue(result["position_ok"] and result["stable_ok"])
        rows = [(i*.02, (0, 0, 0, .3 if i%2 else .5, 0, 0, 20, 0, 0)) for i in range(26)]
        self.assertFalse(capture.evaluate_leg(rows, .5)["stable_ok"])
        with self.assertRaises(RuntimeError):
            capture.evaluate_leg(rows[:3], .5)

    def test_probe_guards(self):
        good = dict(state="4", fault="0", mode="0", target=".5", enc=".4", raw=".2",
                    kp="20", kd=".3", ff="1", i="0", boost="0", age="5", diag="0")
        capture.validate_probe(good, .5)
        for key, value in [("state", "5"), ("fault", "1"), ("mode", "1"),
                           ("i", "1"), ("boost", "1"), ("ff", "0"),
                           ("age", "51"), ("diag", "2"), ("target", "1")]:
            with self.assertRaises(RuntimeError):
                capture.validate_probe({**good, key: value}, .5)

    def test_headers_are_not_samples_and_duplicates_rejected(self):
        def terminal():
            t = capture.Terminal.__new__(capture.Terminal)
            t.mutex = threading.Lock()
            t.file, t.path, t.decoder = io.BytesIO(), Path("offline.log"), None
            t.counts, t.ids = Counter(), {k: set() for k in "ABC"}
            t.capture_invalid, t.expected_count = False, None
            return t
        t = terminal()
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            for line in [b"#YD_BEGIN,v=1,count=2", b"#YD_A_FIELDS,index,x", b"#YD_B_FIELDS,index,x", b"#YD_C_FIELDS,index,x"]:
                t.receive_line(line)
            for index in range(2):
                for kind in "ABC":
                    t.receive_line(f"#YD_{kind},{index},0".encode())
            t.receive_line(b"#YD_END")
        self.assertEqual(dict(t.counts), {"A": 2, "B": 2, "C": 2})
        self.assertNotIn("警告", output.getvalue())
        t = terminal()
        with contextlib.redirect_stdout(io.StringIO()):
            t.receive_line(b"#YD_BEGIN,v=1,count=1")
            t.receive_line(b"#YD_A,0,0")
            t.receive_line(b"#YD_A,0,0")
        self.assertTrue(t.capture_invalid)

    def test_only_explicit_test_command_and_emergency_stop(self):
        t = capture.Terminal.__new__(capture.Terminal)
        t.bench_running = True
        t.bench_abort = threading.Event()
        sent = []
        t._send_raw = sent.append
        with self.assertRaises(RuntimeError):
            t.send("y 10 20")
        t.send("yd")
        self.assertEqual(sent, ["yd"])
        self.assertTrue(t.bench_abort.is_set())
        t.bench_running = False
        started = []
        t.start_bench = started.append
        t.send("ybenchx")
        self.assertEqual(started, [])
        t.send("ybench small")
        self.assertEqual(started, ["ybench small"])

    def test_complete_auto_sequence_and_stale_telemetry_stop(self):
        # Virtual time/serial backend executes the actual runner and leg logic.
        # It verifies sequencing and interlocks, not physical motor dynamics.
        for stale in (False, True):
            with tempfile.TemporaryDirectory(dir=path.parent) as directory:
                t = capture.Terminal.__new__(capture.Terminal)
                t.output_dir = Path(directory)
                t.condition = threading.Condition()
                t.stop = threading.Event()
                t.bench_running, t.bench_failure = True, ""
                t.bench_events, t.bench_writer, t.bench_start = [], None, 0
                t.bench_phase = ""
                t.telemetry = deque()
                clock = dict(now=0., position=0., goal=0., start_position=0., start_time=0., speed=5.)
                commands = []

                def row():
                    values = (20, 20, clock["goal"], clock["position"], 0, 0, 20, .8, 0)
                    t.telemetry.append((clock["now"], values))
                    if t.bench_writer:
                        t.bench_writer.writerow([clock["now"]*1000, t.bench_phase, *values])

                class Abort:
                    def is_set(self): return False
                    def wait(self, delay):
                        for _ in range(3):
                            clock["now"] += .02
                            distance = clock["goal"]-clock["start_position"]
                            duration = max(.2, abs(distance)/clock["speed"])
                            fraction = min(1., (clock["now"]-clock["start_time"])/duration)
                            end = clock["goal"] if distance == 0 else clock["goal"]-(.05 if distance>0 else -.05)
                            clock["position"] = clock["start_position"]+(end-clock["start_position"])*fraction
                            if not stale:
                                row()

                t.bench_abort = Abort()

                def send(command):
                    commands.append(command)
                    parts = command.split()
                    if parts[0] == "y":
                        clock.update(start_position=clock["position"], start_time=clock["now"],
                                     goal=float(parts[1]), speed=float(parts[2]))
                t._send_raw = send
                t.probe = lambda target=None: dict(enc=str(clock["position"]), raw="0.1", target=str(clock["goal"]))
                row()
                with patch.object(capture.time, "monotonic", lambda: clock["now"]):
                    with contextlib.redirect_stdout(io.StringIO()):
                        t.run_bench("all")
                summary_path = next(Path(directory).glob("*.json"))
                import json
                result = json.loads(summary_path.read_text(encoding="utf-8"))
                self.assertEqual(commands[-1], "yd")
                self.assertFalse(t.bench_running)
                if stale:
                    self.assertFalse(result["passed"])
                    self.assertEqual(len([c for c in commands if c.startswith("y ")]), 1)
                else:
                    self.assertTrue(result["passed"])
                    self.assertEqual(len(result["legs"]), 21)
                    self.assertEqual(len([c for c in commands if c.startswith("y ")]), 21)
                    self.assertEqual([r["target_deg"] for r in result["legs"][-6:]], [10, 0, 10, 0, 10, 0])


if __name__ == "__main__":
    unittest.main()
