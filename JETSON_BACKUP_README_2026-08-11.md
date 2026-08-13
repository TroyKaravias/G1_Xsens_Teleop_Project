# G1 Jetson Software-State Milestone

This completes the `2026-08-11` G1 Xsens milestone by copying the software that
the Desktop source snapshot depends on from the Jetson itself.

## Run the collector

Connect the Windows laptop to the G1 by Ethernet. Open PowerShell in this
milestone folder and run:

```powershell
powershell -ExecutionPolicy Bypass -File .\finish_jetson_milestone_2026-08-11.ps1
```

OpenSSH will request the `unitree` password normally. The password is never
stored by either script. Review the sizes printed by the Jetson, ensure that
there is sufficient free space, and type `BACKUP` when prompted.

The operation may take several minutes because the complete GR00T checkout and
Python environment can be large. It does not stop the controller, alter robot
configuration, or overwrite an existing archive.

## Result

After successful completion, this folder contains:

- `G1_Jetson_State_Milestone_2026-08-11.tar.gz`
- `G1_Jetson_State_Milestone_2026-08-11.tar.gz.sha256`

The transfer script verifies the downloaded archive against the Jetson's
SHA-256 checksum. A second copy remains at `/home/unitree/` on the Jetson.

The archive contains:

- `/home/unitree/g1_xsens_direct/g1_xsens_handoff`
- `/home/unitree/GR00T-WholeBodyControl`
- `/home/unitree/.venvs/g1_xsens`
- `/opt/onnxruntime`, when present and readable
- Git state and binary/model checksums
- Package, Python, OS, Jetson, network, and runtime inventory reports

## Boundary

This is a complete backup of the user-space software stack used by the working
teleoperation pipeline. It is not a raw disk image and does not contain the G1
firmware or the Jetson operating-system image. The metadata records those
system versions so the software stack can be restored onto the same compatible
Unitree/Jetson base system.

Do not automatically extract the archive over a working robot. Restoration
should first be performed into a separate directory and compared with the
installed files.
