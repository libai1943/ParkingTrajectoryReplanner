"""Chapter 7: independent connection tasks, collision filtering and three-part stitching.

Chinese book: 非结构化场景自动驾驶轨迹规划技术.
Cite Li et al., IEEE T-IV 7(3):748-757, 2022, doi:10.1109/TIV.2022.3156429.
PolyForm Noncommercial 1.0.0; see ../LICENSE.
"""
from __future__ import annotations

import argparse
import json
import multiprocessing as mp
import time
from pathlib import Path

import numpy as np

from backend import ParkingBackend, ROOT

_worker_backend = None


def initialize_worker(case_id, ready):
    global _worker_backend
    _worker_backend = ParkingBackend(case_id)
    ready.put(True)


def solve_evasion():
    return _worker_backend.evasive()


def solve_connection(task):
    try:
        connector = _worker_backend.connect(task["start"], task["finish"])
        if not _worker_backend.collision_free(connector):
            return None
        return connector
    except RuntimeError:
        return None


def sample(trajectory, timestamp):
    return np.array([np.interp(timestamp, trajectory[:, 0], trajectory[:, j]) for j in range(1, 8)])


def build_tasks(original, evasive, t0, tbrake, think_time=1.2):
    tasks = []
    for i, timestamp in enumerate(np.linspace(t0 + think_time, tbrake, 5), 1):
        start = sample(original, timestamp)
        nearest = np.argmin(np.hypot(evasive[:, 1] - start[0], evasive[:, 2] - start[1]))
        anchor, duration = evasive[nearest, 0], evasive[-1, 0]
        for j, endpoint in enumerate(np.linspace(min(anchor + .05 * duration, duration),
                                                min(anchor + .30 * duration, duration), 6), 1):
            finish = sample(evasive, endpoint)
            finish[2] = start[2] + np.arctan2(np.sin(finish[2] - start[2]), np.cos(finish[2] - start[2]))
            tasks.append(dict(i=i, j=j, start_time=float(timestamp), end_time=float(endpoint),
                              start=start, finish=finish))
    return tasks


def assemble(original, evasive, task, connector, t0):
    prefix = original[(original[:, 0] >= t0) & (original[:, 0] < task["start_time"])].copy()
    if not len(prefix) or prefix[0, 0] > t0:
        prefix = np.vstack(([t0, *sample(original, t0)], prefix))
    suffix = evasive[evasive[:, 0] > task["end_time"]].copy()
    suffix[:, 3] += task["finish"][2] - sample(evasive, task["end_time"])[2]
    suffix[:, 0] += task["start_time"] + connector[-1, 0] - task["end_time"]
    middle = connector.copy()
    middle[:, 0] += task["start_time"]
    return np.vstack((prefix, middle, suffix))


def validate(backend, trajectory, task, connector):
    if not np.all(np.isfinite(trajectory)) or not np.all(np.isfinite(connector)):
        return dict(passed=False, reason="nonfinite_result")
    z, dt = connector[:, 1:], np.diff(connector[:, 0])[:, None]
    rhs = np.column_stack((z[:-1, 3] * np.cos(z[:-1, 2]), z[:-1, 3] * np.sin(z[:-1, 2]),
                           z[:-1, 3] * np.tan(z[:-1, 5]) / 2.8, z[:-1, 4], z[:-1, 6]))
    dynamics = float(np.max(np.abs(np.diff(z[:, [0, 1, 2, 3, 5]], axis=0) - dt * rhs)))
    join = float(max(np.max(np.abs(z[0] - task["start"])), np.max(np.abs(z[-1] - task["finish"]))))
    violation = float(max(0, np.max(np.abs(z[:, 3:]) - [3, 2, .85, .7])))
    free = backend.collision_free(trajectory)
    monotonic = bool(np.all(np.diff(trajectory[:, 0]) > 0))
    return dict(join_error=join, dynamics_error=dynamics, bound_violation=violation,
                collision_free=free, time_increasing=monotonic,
                minimum_join_speed=float(min(abs(task["start"][3]), abs(task["finish"][3]))),
                passed=join < 1e-5 and dynamics < 1e-5 and violation < 1e-5 and free and monotonic)


def brake(original, t0, timestamp):
    state = sample(original, timestamp)
    prefix = original[(original[:, 0] >= t0) & (original[:, 0] < timestamp)]
    rows = [[timestamp, *state]]
    while abs(state[3]) > 1e-10:
        dt, acceleration = min(.01, abs(state[3]) / 2), -np.sign(state[3]) * 2
        state[0] += dt * state[3] * np.cos(state[2])
        state[1] += dt * state[3] * np.sin(state[2])
        state[2] += dt * state[3] * np.tan(state[5]) / 2.8
        state[3] += dt * acceleration
        state[4], state[6] = acceleration, 0
        timestamp += dt
        rows.append([timestamp, *state])
    rows[-1][5] = 0
    return np.vstack((prefix, rows))


def run(case_id=1, workers=4, enforce_deadline=False):
    backend = ParkingBackend(case_id)
    original = backend.original()
    t0, t1, tbrake, _ = backend.metadata
    think_time = 1.2
    report = dict(case_id=case_id, t0=float(t0), t1=float(t1), tbrake=float(tbrake),
                  workers=workers, enforce_deadline=enforce_deadline, completed=0, valid_count=0)
    if tbrake < t0 + think_time:
        trajectory = brake(original, t0, t0)
        report.update(status="fail_safe_insufficient_window", wall_seconds=0.,
                      collision_free=backend.collision_free(trajectory))
        backend.close()
        return trajectory, report
    context = mp.get_context("spawn")
    ready = context.Queue()
    pool = context.Pool(workers, initialize_worker, (case_id, ready))
    for _ in range(workers):
        ready.get(timeout=90)  # Prepare the execution resources before t0.
    started = time.perf_counter()
    best = None
    try:
        evasive_job = pool.apply_async(solve_evasion)
        try:
            evasive = evasive_job.get(timeout=think_time if enforce_deadline else None)
        except (mp.TimeoutError, RuntimeError) as error:
            report.update(status="fail_safe_evasion_or_deadline", detail=str(error))
            evasive = None
        if evasive is not None:
            tasks = build_tasks(original, evasive, t0, tbrake, think_time)
            pending = [(task, pool.apply_async(solve_connection, (task,))) for task in tasks]
            while pending:
                if enforce_deadline and time.perf_counter() - started >= think_time:
                    report["deadline_reached"] = True
                    break
                for task, job in pending.copy():
                    if not job.ready():
                        continue
                    pending.remove((task, job))
                    report["completed"] += 1
                    connector = job.get()
                    if connector is None:
                        continue
                    candidate = assemble(original, evasive, task, connector, t0)
                    validation = validate(backend, candidate, task, connector)
                    if not validation["passed"]:
                        continue
                    report["valid_count"] += 1
                    cost = task["start_time"] - t0 + connector[-1, 0] + evasive[-1, 0] - task["end_time"]
                    if best is None or cost < best[0]:
                        best = (cost, candidate, task, validation)
                time.sleep(.002)
            if best is None:
                report["status"] = "fail_safe_no_connection"
        if best is None:
            trajectory = brake(original, t0, min(t0 + think_time, tbrake))
            report["collision_free"] = backend.collision_free(trajectory)
        else:
            cost, trajectory, task, validation = best
            report.update(status="stitched", remaining_time=float(cost), selected_pair=[task["i"], task["j"]],
                          validation=validation)
        report["wall_seconds"] = time.perf_counter() - started
        return trajectory, report
    finally:
        pool.terminate()
        pool.join()
        ready.close()
        backend.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", type=int, default=1, choices=[1, 14, 20, 36, 39, 96, 100, 108])
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--deadline", action="store_true", help="Enforce the paper's 1.2 s online computation budget.")
    parser.add_argument("--output", type=Path, default=ROOT / "python" / "results")
    args = parser.parse_args()
    if args.workers < 1 or args.workers > 32:
        parser.error("--workers must lie between 1 and 32")
    trajectory, report = run(args.case, args.workers, args.deadline)
    args.output.mkdir(parents=True, exist_ok=True)
    np.savetxt(args.output / "trajectory.csv", trajectory, delimiter=",",
               header="t,x,y,theta,v,a,phi,omega", comments="")
    (args.output / "report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    mp.freeze_support()
    main()
