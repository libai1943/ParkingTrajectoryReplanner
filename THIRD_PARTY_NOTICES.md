# External components

This repository does not redistribute MathWorks, AMPL, IPOPT, HSL, or CasADi binaries. Install them from their respective providers and retain their licenses.

| Component | Use | Official information |
| --- | --- | --- |
| MATLAB, Navigation Toolbox, Image Processing Toolbox | MATLAB execution, Reeds–Shepp and cost map | [MathWorks licensing](https://www.mathworks.com/company/aboutus/policies_statements.html) |
| AMPL and its IPOPT executable | MATLAB text-file solver interface | [AMPL installation](https://dev.ampl.com/ampl/install.html), [IPOPT](https://dev.ampl.com/solvers/ipopt/index.html) |
| CasADi 3.7.2 | External dynamic dependency of our native backend | [CasADi documentation and LGPL license information](https://web.casadi.org/docs/) |
| IPOPT and MUMPS supplied by the CasADi package | Native nonlinear optimization and linear solves | [IPOPT project](https://github.com/coin-or/Ipopt) |
| GCC runtime and libstdc++ | Linked into the supplied MinGW-built project DLL | [GCC Runtime Library Exception](https://www.gnu.org/licenses/gcc-exception-3.1.html) |

`native/win64/parking_backend.dll` is this project's own compiled backend, dynamically linked to the separately installed CasADi library. CasADi may be replaced with an ABI-compatible build; it is not embedded into this DLL. Its use does not relicense CasADi or its dependencies under this project's noncommercial terms.

No third-party Arrow plotting implementation from the old working directory is included in this release. The consolidated renderer is project code.

For scientific use of CasADi, also follow the citation request in the official CasADi documentation.
