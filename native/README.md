# Protected parking backend

`win64/parking_backend.dll` is this project's compiled Windows x64 component.

- Runtime: **CasADi 3.7.2 Windows x64**, installed separately.
- Direct PE imports: `KERNEL32.dll`, `msvcrt.dll`, `libcasadi.dll`.
- CasADi is dynamically linked; its binaries and source are not embedded.
- Project license: [PolyForm Noncommercial 1.0.0](../LICENSE).
- SHA-256: `8a59b6830f1eb70b9c65b7fdfc18815d4247d14f9654e6d87983629bdcb384e0`.

The public C ABI accepts real `double` arrays in row-major order. Trajectory rows are `[t,x,y,theta,v,a,phi,omega]`; boundary states omit `t`. Each worker process owns its backend instance. This build provides dataset loading, initial trajectory search, LIOM, connector optimization and geometric checks. Visualization is not included.

| Export | Contract |
| --- | --- |
| `ps_load(path_utf8, case_id)` | Load packed MAT v4 scene; return opaque handle or null |
| `ps_free(handle)` | Release scene |
| `ps_error()` | Last error string in the calling thread |
| `ps_metadata(handle, out4)` | Write `t0,t1,tbrake,original_duration` |
| `ps_original(handle, out, capacity)` | Copy original trajectory; null `out` queries row count |
| `ps_evasive(handle, out, capacity)` | Compute 100-point evasive trajectory; negative return means failure |
| `ps_connect(start7, end7, out, capacity)` | Compute 100-point connector; negative return means failure |
| `ps_collision_free(handle, rows, count)` | Check rectangular footprints at supplied configurations |

The backend implementation source is intentionally not distributed. The readable C++ and Python orchestration code calls this interface. No Linux/macOS binary is supplied in this release.
