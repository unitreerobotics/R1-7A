"""Timing diagnostics use measured intervals, never the requested frequency."""
import unittest

from teleop.control_timing import TimingReporter, timing_summary


class ControlTiming(unittest.TestCase):
    def test_measured_rate_and_long_gap_are_visible(self):
        rows = [{'cycle': .0125, 'right_command_interval': .0125,
                 'left_command_interval': .0125, 'overruns': i} for i in range(8)]
        rows[-1]['left_command_interval'] = .025
        report = timing_summary(rows, 90.)
        self.assertEqual(report['target_hz'], 90.)
        self.assertEqual(report['control_hz'], 80.)
        self.assertEqual(report['right_command_hz'], 80.)
        self.assertAlmostEqual(report['left_command_hz'], 71.11, places=2)
        self.assertEqual(report['left_command_interval_max_ms'], 25.)
        self.assertEqual(report['overruns'], 7)

    def test_first_missing_command_interval_and_dry_run(self):
        report = timing_summary([{'cycle': .01},
                                 {'cycle': .01, 'right_command_interval': .02}], 90.)
        self.assertEqual(report['right_command_hz'], 50.)
        self.assertNotIn('left_command_hz', report)
        dry = timing_summary([{'cycle': .01, 'right_io': 0.}], 90.)
        self.assertNotIn('right_command_hz', dry)

    def test_disabled_reporter_starts_no_thread(self):
        reporter = TimingReporter(False)
        self.assertIsNone(reporter.thread)
        reporter.add({'cycle': .01})
        self.assertEqual(len(reporter.samples), 0)
        reporter.close()


if __name__ == '__main__':
    unittest.main()
