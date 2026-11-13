#!/usr/bin/env python3
"""Exercise incremental links with the real CMake command dependency lists.

Only a tiny native library is compiled. A stand-in linker hashes its actual
archive and glue inputs; no SDK, Jolt build or network access is needed.
"""
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import unittest


SOURCE = Path(__file__).resolve().parents[1] / 'CMakeLists.txt'


class LinkDependencies(unittest.TestCase):
    def exercise(self, generator, remove_dependencies=False):
        blocks = [match.group(0) for match in re.finditer(
            r'add_custom_command\(\s*\n.*?\n[ \t]*VERBATIM\)', SOURCE.read_text(), re.S)
            if 'COMMAND emcc glue.o $<TARGET_FILE:Jolt>' in match.group(0)]
        self.assertEqual(len(blocks), 3, 'all JS, Wasm and compat link commands')
        if remove_dependencies:
            blocks = [block.replace('jolt-bindings Jolt glue.o glue.js ', 'jolt-bindings ') for block in blocks]
        # Keep scratch beside this checkout rather than a system temporary disk.
        with tempfile.TemporaryDirectory(prefix='link-dependency-test-', dir=SOURCE.parent.parent) as directory:
            root = Path(directory)
            (root / 'jolt.c').write_text('int jolt_value(void) { return 1; }\n')
            for name in ['glue-input.o', 'glue-input.js', 'header.h', 'jolt.idl', 'front.js']:
                (root / name).write_text(name + '\n')
            (root / 'link.py').write_text('''import hashlib,json,sys
from pathlib import Path
archive=Path(sys.argv[1]); output=Path(sys.argv[sys.argv.index('-o')+1])
digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
record=json.dumps([digest(archive),digest(Path('glue.o')),digest(Path('glue.js'))])
output.write_text(record)
if output.name.endswith('.wasm.js'):output.with_suffix('.wasm').write_text(record)
with Path('links.jsonl').open('a') as log:log.write(output.name+'\\n')
''')
            commands = '\n'.join(block.replace('COMMAND emcc glue.o',
                'COMMAND "${Python3_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/link.py"') for block in blocks)
            (root / 'CMakeLists.txt').write_text('''cmake_minimum_required(VERSION 3.20)
project(LinkDependencies LANGUAGES C)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
add_library(Jolt STATIC jolt.c)
set(OUTPUT_BASE_NAME "${CMAKE_BINARY_DIR}/jolt")
set(JOLT_HEADER_FILE "${CMAKE_SOURCE_DIR}/header.h")
set(JOLT_IDL_FILE "${CMAKE_SOURCE_DIR}/jolt.idl")
set(JOLT_FRONT_MATTER_FILE "${CMAKE_SOURCE_DIR}/front.js")
set(REPLACE_IMPORT "${CMAKE_COMMAND}" -E true)
configure_file(glue-input.o glue.o COPYONLY)
configure_file(glue-input.js glue.js COPYONLY)
add_custom_target(jolt-bindings DEPENDS glue.o glue.js)
''' + commands + '''
add_custom_target(all-links ALL DEPENDS "${OUTPUT_BASE_NAME}.js"
    "${OUTPUT_BASE_NAME}.wasm.js" "${OUTPUT_BASE_NAME}.wasm.wasm"
    "${OUTPUT_BASE_NAME}.wasm-compat.js")
''')
            build = root / 'build'

            def run(*args):
                subprocess.run(args, check=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

            def rebuild():
                run('cmake', '--build', str(build), '--parallel', '1')

            def linked_records():
                return [json.loads((build / file).read_text()) for file in
                    ['jolt.js', 'jolt.wasm.js', 'jolt.wasm.wasm', 'jolt.wasm-compat.js']]

            def link_count():
                return len((build / 'links.jsonl').read_text().splitlines())

            run('cmake', '-G', generator, '-S', str(root), '-B', str(build))
            rebuild()
            before = linked_records()
            self.assertEqual(link_count(), 3)
            rebuild()
            self.assertEqual(link_count(), 3, 'unchanged build must not relink')
            (root / 'jolt.c').write_text('int jolt_value(void) { return 2; }\n')
            rebuild()
            after = linked_records()
            archive = hashlib.sha256((build / 'libJolt.a').read_bytes()).hexdigest()
            self.assertNotEqual(archive, before[0][0], 'native fixture really rebuilt')
            self.assertTrue(all(row[0] == archive for row in after), 'native archive must relink all outputs')
            self.assertEqual(link_count(), 6)
            for index, name in [(1, 'glue.o'), (2, 'glue.js')]:
                (build / name).write_text('changed ' + name + '\n')
                rebuild()
                expected = hashlib.sha256((build / name).read_bytes()).hexdigest()
                self.assertTrue(all(row[index] == expected for row in linked_records()), name + ' must relink all outputs')
            self.assertEqual(link_count(), 12)
            rebuild()
            self.assertEqual(link_count(), 12, 'final unchanged build must not relink')

    def test_actual_commands_relink(self):
        for generator in ['Unix Makefiles', 'Ninja']:
            if generator == 'Ninja' and not shutil.which('ninja'):
                continue
            with self.subTest(generator=generator):
                self.exercise(generator)

    def test_missing_dependencies_is_detected(self):
        with self.assertRaisesRegex(AssertionError, 'native archive must relink'):
            self.exercise('Unix Makefiles', remove_dependencies=True)


if __name__ == '__main__':
    unittest.main()
