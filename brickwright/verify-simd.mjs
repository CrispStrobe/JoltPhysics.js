// Every original physical control must pass for both independent build flavors.
import assert from 'node:assert/strict';
import {spawnSync} from 'node:child_process';
import {readFileSync,existsSync} from 'node:fs';
const run = flavor => {
    const file = `dist/jolt-physics.${flavor}.js`;
    assert.ok(existsSync(file),`Missing shipped ${file}`);
    const result = spawnSync(process.execPath,['brickwright/verify.mjs',file,'--assert','--json'],{encoding:'utf8'});
    process.stdout.write(result.stdout||'');process.stderr.write(result.stderr||'');
    assert.equal(result.error,undefined);assert.equal(result.status,0,`${flavor} physical controls`);
    assert.match(result.stdout,/\nASSERT OK\s*$/);
    return JSON.parse(result.stdout.replace(/\nASSERT OK\s*$/,''));
};
const baseline=run('wasm-compat'),simd=run('simd.wasm-compat');
assert.equal(simd.results.regression.hash,baseline.results.regression.hash,'SIMD must preserve the original regression hash');
assert.equal(simd.results.regression.hash,'f7b0121879392938','unchanged qualified non-gear regression');
const pkg=JSON.parse(readFileSync('package.json','utf8'));
for(const file of Object.values(pkg.exports['./wasm-simd-compat'])){
    assert.ok(existsSync(file),`Missing exported ${file}`);
    assert.ok(pkg.files.includes(file),`Export omitted from npm package: ${file}`);
}
console.log('Both flavors pass the original physical controls and exact regression hash.');
