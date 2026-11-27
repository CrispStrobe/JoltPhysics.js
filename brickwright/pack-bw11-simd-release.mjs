// SPDX-License-Identifier: MIT
import assert from 'node:assert/strict';
import {execFileSync} from 'node:child_process';
import {createHash} from 'node:crypto';
import {copyFileSync, mkdtempSync, readFileSync, rmSync, writeFileSync} from 'node:fs';
import {tmpdir} from 'node:os';
import {join, resolve} from 'node:path';

assert.equal(process.argv.length, 5,
    'Usage: node brickwright/pack-bw11-simd-release.mjs <bw.10-tarball> <SIMD-module> <output-directory>');
const [baseArchive, simdModule, output] = process.argv.slice(2).map(path => resolve(path));
const baseIntegrity = 'fyGyoLpkz1FIjEoc8movWT4N5Vj+iME2gTANV8jbzm7UMUH8B5K1y0EWboJibkFUisK5kS1LjVaT3xZ7fE3RCA==';
const simdSha256 = '3c463978c83694b04a1e4a78c1611f488e195ac087fb5d30a572a4ed10b80af9';
const hash = (path, algorithm, format) => createHash(algorithm).update(readFileSync(path)).digest(format);
assert.equal(hash(baseArchive, 'sha512', 'base64'), baseIntegrity, 'qualified bw.10 archive');
assert.equal(hash(simdModule, 'sha256', 'hex'), simdSha256, 'qualified SIMD module');
const manifest = JSON.parse(readFileSync(new URL('../package.json', import.meta.url), 'utf8'));
assert.equal(manifest.version, '1.1.0-bw.11');

const stage = mkdtempSync(join(tmpdir(), 'jolt-bw11-pack-'));
try {
    execFileSync('tar', ['-xzf', baseArchive, '-C', stage]);
    const pkg = join(stage, 'package');
    const manifestPath = join(pkg, 'package.json');
    const base = JSON.parse(readFileSync(manifestPath, 'utf8'));
    assert.equal(base.version, '1.1.0-bw.10');
    assert.deepEqual(manifest.exports, base.exports);
    copyFileSync(simdModule, join(pkg, manifest.exports['./wasm-simd-compat'].default));
    copyFileSync(new URL('../BRICKWRIGHT.md', import.meta.url), join(pkg, 'BRICKWRIGHT.md'));
    writeFileSync(manifestPath, JSON.stringify(manifest, null, 2) + '\n');
    process.stdout.write(execFileSync('npm', [
        'pack', '--ignore-scripts', '--json', '--pack-destination', output, pkg,
    ], {encoding: 'utf8'}));
} finally {
    rmSync(stage, {recursive: true, force: true});
}
