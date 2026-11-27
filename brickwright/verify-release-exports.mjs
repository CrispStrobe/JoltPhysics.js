// SPDX-License-Identifier: MIT
import assert from 'node:assert/strict';
import {spawnSync} from 'node:child_process';
import {mkdirSync, mkdtempSync, readFileSync, rmSync, statSync, symlinkSync, writeFileSync} from 'node:fs';
import {tmpdir} from 'node:os';
import {join, resolve} from 'node:path';

assert.equal(process.argv.length, 3, 'Usage: node brickwright/verify-release-exports.mjs <unpacked-package>');
const root = resolve(process.argv[2]);
const manifest = JSON.parse(readFileSync(resolve(root, 'package.json'), 'utf8'));
assert.equal(manifest.name, 'jolt-physics');

const trial = mkdtempSync(join(tmpdir(), 'jolt-export-check-'));
try {
    mkdirSync(join(trial, 'node_modules'));
    symlinkSync(root, join(trial, 'node_modules', manifest.name), 'dir');
    const probe = join(trial, 'probe.mjs');

    for (const [subpath, entry] of Object.entries(manifest.exports)) {
        const files = typeof entry === 'string' ? [entry] : Object.values(entry);
        for (const file of files) {
            const info = statSync(resolve(root, file));
            assert.ok(info.isFile() && info.size > 0, `Missing export payload ${file}`);
        }
        const target = typeof entry === 'string' ? entry : entry.default;
        if (!target.endsWith('.js')) continue;

        const specifier = subpath === '.' ? manifest.name : manifest.name + subpath.slice(1);
        const code = `
            import assert from 'node:assert/strict';
            const Jolt = (await import(${JSON.stringify(specifier)})).default;
            const J = await Jolt();
            const point = new J.Vec3(1, 2, 3);
            assert.deepEqual([point.GetX(), point.GetY(), point.GetZ()], [1, 2, 3]);
            J.destroy(point);
            process.exit(0);
        `;
        // Each pthread build owns its own pool and WebAssembly instance.
        writeFileSync(probe, code);
        const child = spawnSync(process.execPath, [probe], {
            encoding: 'utf8', timeout: 60_000,
        });
        if (child.error) throw child.error;
        assert.equal(child.status, 0, `${specifier} failed to initialize\n${child.stderr.slice(-4000)}`);
        console.log(`PASS ${specifier}`);
    }
} finally {
    rmSync(trial, {recursive: true, force: true});
}
