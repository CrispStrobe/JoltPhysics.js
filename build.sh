#!/bin/sh
set -e

# Honor an explicit parallelism cap for local builds.
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-$(getconf _NPROCESSORS_ONLN)}"

if [ -z $1 ] 
then
	BUILD_TYPE=Distribution
else
	BUILD_TYPE=$1
	shift
fi

rm -rf ./dist

mkdir dist

if [ $BUILD_TYPE != "Debug" ]
then
	cmake -B Build/Debug/ST -DCMAKE_BUILD_TYPE=Debug -DBUILD_WASM_COMPAT_ONLY=ON "${@}"
	cmake --build Build/Debug/ST

	cmake -B Build/Debug/MT -DENABLE_MULTI_THREADING=ON -DENABLE_SIMD=ON -DCMAKE_BUILD_TYPE=Debug -DBUILD_WASM_COMPAT_ONLY=ON "${@}"
	cmake --build Build/Debug/MT

	mv ./dist/jolt-physics.wasm-compat.js ./dist/jolt-physics.debug.wasm-compat.js
	mv ./dist/jolt-physics.multithread.wasm-compat.js ./dist/jolt-physics.debug.multithread.wasm-compat.js
fi

cmake -B Build/$BUILD_TYPE/ST -DCMAKE_BUILD_TYPE=$BUILD_TYPE "${@}"
cmake --build Build/$BUILD_TYPE/ST

cmake -B Build/$BUILD_TYPE/MT -DENABLE_MULTI_THREADING=ON -DENABLE_SIMD=ON -DCMAKE_BUILD_TYPE=$BUILD_TYPE "${@}"
cmake --build Build/$BUILD_TYPE/MT

if [ $BUILD_TYPE = "Debug" ]
then
	cp ./dist/jolt-physics.wasm-compat.js ./dist/jolt-physics.debug.wasm-compat.js
	cp ./dist/jolt-physics.multithread.wasm-compat.js ./dist/jolt-physics.debug.multithread.wasm-compat.js
fi

# Optional single-thread SIMD: no shared memory or cross-origin isolation.
# Keep the existing scalar and multithread artifacts at their original names.
cmake -B Build/$BUILD_TYPE/ST-SIMD -DENABLE_MULTI_THREADING=OFF -DENABLE_SIMD=ON -DBUILD_WASM_COMPAT_ONLY=ON -DCMAKE_BUILD_TYPE=$BUILD_TYPE "${@}"
cmake --build Build/$BUILD_TYPE/ST-SIMD

# Update the worker URL in the copied wasm-compat.js files
debug_mt=./dist/jolt-physics.debug.multithread.wasm-compat.js
sed 's:jolt-physics.multithread.wasm-compat.js:jolt-physics.debug.multithread.wasm-compat.js:g' "$debug_mt" > "$debug_mt.tmp"
mv "$debug_mt.tmp" "$debug_mt"

cat > ./dist/jolt-physics.d.ts << EOF
import Jolt from "./types";

export default Jolt;
export * from "./types";

EOF

cp ./dist/jolt-physics.d.ts ./dist/jolt-physics.wasm.d.ts
cp ./dist/jolt-physics.d.ts ./dist/jolt-physics.wasm-compat.d.ts
cp ./dist/jolt-physics.d.ts ./dist/jolt-physics.simd.wasm-compat.d.ts
cp ./dist/jolt-physics.d.ts ./dist/jolt-physics.debug.wasm-compat.d.ts
cp ./dist/jolt-physics.d.ts ./dist/jolt-physics.multithread.d.ts
cp ./dist/jolt-physics.d.ts ./dist/jolt-physics.multithread.wasm.d.ts
cp ./dist/jolt-physics.d.ts ./dist/jolt-physics.multithread.wasm-compat.d.ts
cp ./dist/jolt-physics.d.ts ./dist/jolt-physics.debug.multithread.wasm-compat.d.ts

cp ./dist/jolt-physics*.wasm-compat.js ./Examples/js/
