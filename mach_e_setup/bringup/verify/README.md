# Drive verification watchers (2026-10-15 fixes)
Run each in its own terminal (or let Claude run them as monitors) BEFORE launching `run_mach_e.sh`:
- `watch_run.sh`      waits for the next run dir in ~/autoware_logs/runs, records a planning/control bag to
                      /mnt/mapdata/autoware_runs/<run>_planning after 60 s, prints deduped ERROR/FATAL/died/out-of-memory
                      lines and GPU status every 5 min (alarm > 12 GB).
- `verify_mrm.py`     MRM episodes while engaged + 2-min summary (EKF fix: expect 0).
- `stopline_watch.py` mode changes, stop_line planning factor on/off, every crossing of a PD2LAB stop line (OK / OVERRUN).
- `diag_sample.py N`  N-second sample of diagnostics_graph items at WARN or worse + autonomous availability.
All need `source ~/utils/autoware_env_setup/aw.sh` first.
