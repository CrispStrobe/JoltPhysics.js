# Components in the WebAssembly package

- JoltPhysics.js and its Brickwright modifications: MIT. The original notice
  remains in `LICENSE`. Upstream: https://github.com/jrouwe/JoltPhysics.js;
  modified source: https://github.com/CrispStrobe/JoltPhysics.js.
- Jolt Physics and its Brickwright modifications: MIT. The full original
  notice is retained in `LICENSES/JoltPhysics.txt`. Upstream:
  https://github.com/jrouwe/JoltPhysics; modified source:
  https://github.com/CrispStrobe/JoltPhysics. The native source commit used by
  the binding build is pinned in `CMakeLists.txt`.
- Emscripten generated runtime support: MIT or University of Illinois/NCSA.
  The full original dual-license notice from Emscripten 6.0.2 is retained in
  `LICENSES/Emscripten.txt`. Source: https://github.com/emscripten-core/emscripten.

Build tools are not bundled in this package. Preserve these notices when
redistributing the corresponding code. Package qualification is scoped to its
recorded source and artifacts; these notices do not assert blanket legal clearance.


The qualified Emscripten 6.0.2 runtime inputs also retain their original notices:

- `LICENSES/Musl.txt`: `system/lib/libc/musl/COPYRIGHT`.
- `LICENSES/LLVM-libcxx.txt`: `system/lib/libcxx/LICENSE.TXT`.
- `LICENSES/LLVM-libcxxabi.txt`: `system/lib/libcxxabi/LICENSE.TXT`.
- `LICENSES/LLVM-compiler-rt.txt`: `system/lib/compiler-rt/LICENSE.TXT`.

These are exact notice copies from the SDK source inputs. Retain them alongside
the Emscripten and Jolt notices when redistributing the generated modules.
