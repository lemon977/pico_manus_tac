# Device camera calibration

Place one directory per PICO serial number here:

```text
config/pico_cam/devices/<PICO_SERIAL>/camera_params.json
```

Real serial numbers and calibration files are intentionally ignored by Git. The
collector snapshots the matching file into each local session. Never commit a
device directory or a captured `camera_params.json` containing a headset serial.
