import time
import numpy as np

class LatencyTracker:
    def __init__(self):
        self.queue_L = []
        self.queue_R = []
        self.proc = []
        self.obj = []
        self.e2e = []
        self.start_time = time.perf_counter()
        self.frame_times = []
        
    def record(self, tL, tR, t_read, t_proc_start, t_proc_end):
        now = time.perf_counter()
        self.frame_times.append(now)
        
        qL = (t_read - tL) * 1000
        qR = (t_read - tR) * 1000
        proc = (t_proc_end - t_proc_start) * 1000
        e2e = max(qL, qR) + proc
        
        self.queue_L.append(qL)
        self.queue_R.append(qR)
        self.proc.append(proc)
        self.e2e.append(e2e)
        
    def as_arrays(self):
        return (
            np.array(self.queue_L),
            np.array(self.queue_R),
            np.array(self.proc),
            np.array(self.e2e)
        )
        
    def stats(self, arr):
        return {
            "mean": float(arr.mean()),
            "std": float(arr.std()),
            "min": float(arr.min()),
            "max": float(arr.max()),
            "p50": float(np.percentile(arr, 50)),
            "p90": float(np.percentile(arr, 90)),
            "p99": float(np.percentile(arr, 99)),
        }
        
    def summary(self):
        qL, qR, proc, e2e = self.as_arrays()
        
        return {
            "queue_L": self.stats(qL),
            "queue_R": self.stats(qR),
            "proc": self.stats(proc),
            "e2e": self.stats(e2e)
        }
        
    def count(self):
        return len(self.e2e)
    
    def elapsed(self):
        return time.perf_counter() - self.start_time
    
    def elapsed_hms(self):
        return self.format_hms(self.elapsed())

    def reset(self):
        self.queue_L.clear()
        self.queue_R.clear()
        self.proc.clear()
        self.e2e.clear()
        self.start_time = time.perf_counter()

    def fps(self):
        if not self.e2e:
            return 0.0
        return 1000.0 / np.mean(self.e2e)
    
    def fps_wallclock(self):
        elapsed = self.elapsed()
        if elapsed == 0:
            return 0.0
        return self.count() / elapsed
    
    def fps_camera(self):
        """Estimate FPS based on actual frame arrival times"""
        if len(self.frame_times) < 2:
            return 0.0
        intervals = np.diff(self.frame_times)  # seconds between frames
        mean_interval = np.mean(intervals)
        return 1.0 / mean_interval

        
    @staticmethod
    def print_stats(name, stats):
        if stats is None:
            print(f"{name}: no data")
            return

        print(
            f"{name}: "
            f"mean={stats['mean']:.1f} ms | "
            f"median={stats['p50']:.1f} ms | "
            f"std={stats['std']:.1f} ms | "
            f"min={stats['min']:.1f} ms | "
            f"max={stats['max']:.1f} ms | "
            f"90th={stats['p90']:.1f} ms | "
            f"99th={stats['p99']:.1f} ms"
        )
        
    @staticmethod
    def format_hms(seconds):
        hours = int(seconds // 3600)
        minutes = int((seconds % 3600) // 60)
        secs = int(seconds % 60)
        return f"{hours:02d}:{minutes:02d}:{secs:02d}"
    
        
    def print_summary(self):
        summary = self.summary()

        print("\n--- Latency Statistics ---")
        self.print_stats("E2E Latency", summary["e2e"])
        self.print_stats("Processing Time", summary["proc"])
        self.print_stats("Queue Delay L", summary["queue_L"])
        self.print_stats("Queue Delay R", summary["queue_R"])

        print(f"\nFrames recorded: {self.count()}")
        print(f"FPS (latency): {self.fps():.2f}")
        print(f"FPS (wall-clock): {self.fps_wallclock():.2f}")
        print(f"FPS (camera): {self.fps_camera():.2f}")
        print(f"Elapsed time: {self.elapsed_hms()}")
