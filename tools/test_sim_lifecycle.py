#!/usr/bin/env python3
"""Exercise real isolated process groups; never start or signal a real simulator."""
import os
import signal
import select
import subprocess
import sys
import unittest

from run_rate_identification import stop_sim


@unittest.skipUnless(sys.platform.startswith('linux'), 'Linux process-group lifecycle')
class SimulatorLifecycle(unittest.TestCase):
    def test_reaps_signal_resistant_descendant_without_stopping_other_group(self):
        child = ('import os,signal,time; '
                 'signal.signal(signal.SIGINT,signal.SIG_IGN); '
                 'signal.signal(signal.SIGTERM,signal.SIG_IGN); '
                 'print(os.getpid(),flush=True); time.sleep(60)')
        parent = f'import subprocess,sys,time; subprocess.Popen([sys.executable,"-c",{child!r}]); time.sleep(60)'
        sim = subprocess.Popen([sys.executable,'-c',parent],start_new_session=True,
                               stdout=subprocess.PIPE,stderr=subprocess.DEVNULL,text=True)
        unrelated = subprocess.Popen([sys.executable,'-c','import time; time.sleep(60)'],
                                     start_new_session=True)
        try:
            # Child prints only after its signal handlers are installed.
            self.assertTrue(select.select([sim.stdout],[],[],5.)[0], 'child startup timed out')
            descendant = int(sim.stdout.readline())
            stop_sim(sim)
            sim.wait(timeout=2)
            probe = subprocess.run(['ps','-o','stat=','-p',str(descendant)],capture_output=True,text=True)
            self.assertTrue(not probe.stdout.strip() or probe.stdout.strip().startswith('Z'))
            self.assertIsNone(unrelated.poll())
        finally:
            for process in (sim,unrelated):
                try:
                    os.killpg(process.pid,signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait(timeout=2)
            sim.stdout.close()


if __name__=='__main__':
    unittest.main()
