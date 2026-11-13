#!/usr/bin/env python3
import importlib.util
from pathlib import Path
import unittest
spec=importlib.util.spec_from_file_location('repair',Path(__file__).with_name('fix-node-pthread-footer.py'));module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
PREFIX=b'function Jolt(){return 42;}export default Jolt;'
FOOTER=b'var isPthread=false;var isNode=true;if(isNode)isPthread='+module.BAD[0]+b';isPthread&&Jolt();'
class Controls(unittest.TestCase):
 def test_regression_and_body(self):
  result,status=module.repair(PREFIX+FOOTER);self.assertEqual(status,'corrected');self.assertEqual(result.split(module.EXPORT)[0],PREFIX.split(module.EXPORT)[0]);self.assertNotIn(module.BAD[0],result);self.assertIn(module.GOOD[0],result)
 def test_idempotent(self):
  good,_=module.repair(PREFIX+FOOTER);self.assertEqual(module.repair(good),(good,'already-correct'))
 def test_real_debug_shape(self):
  footer=b"\nvar isPthread = globalThis.name?.startsWith('em-pthread');\nvar isNode = true;\nif (isNode) isPthread = (await import('node:worker_threads')).workerData === 'em-pthread'\n\nisPthread && Jolt();\n"
  self.assertEqual(module.repair(PREFIX+footer),(PREFIX+footer,'already-correct'))
 def test_ambiguous_refuses(self):
  for extra in [module.BAD[0],module.GOOD[0]]:
   with self.assertRaises(ValueError):module.repair(PREFIX+FOOTER.replace(b';isPthread&&Jolt();',b';'+extra+b';isPthread&&Jolt();'))
 def test_unknown_or_missing_export_refuses(self):
  for data in [FOOTER,PREFIX+FOOTER+module.EXPORT,PREFIX+b'unknown;']:
   with self.assertRaises(ValueError):module.repair(data)
if __name__=='__main__':unittest.main()
