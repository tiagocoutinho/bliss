import pytest
import os

test_cfg = """name: rw_test
test: this is just a test.
# this is a comment!
one:
- pink: martini
  red: apples #this one will change
  blue: berry
two:
- a: true
  b: false
  c: null
"""

test_cfg2 = """name: rw_test
test: this is just a test.
# this is a comment!
one:
- pink: martini
  red: strawberry #this one will change
  blue: berry
two:
- a: true
  b: false
  c:
"""

TEST_FILE_PATH = os.path.join(os.path.dirname(__file__), "..", "test_configuration", "read_write.yml")

@pytest.mark.xfail
def test_config_save(beacon):
  rw_cfg = beacon.get_config("rw_test")

  # would be better to ask beacon to get file contents dump
  with open(TEST_FILE_PATH, "r") as f:
    assert f.read() == test_cfg

  rw_cfg['one'][0]['red'] = 'strawberry'

  rw_cfg.save()

  beacon.reload()

  rw_cfg2 = beacon.get_config("rw_test")

  assert rw_cfg2['one'][0]['red'] == 'strawberry'

  try:
      with open(TEST_FILE_PATH, "r") as f:
          assert f.read() == test_cfg2
  finally:
      rw_cfg2['one'][0]['red'] = 'apples'
  
      rw_cfg2.save()

def test_references(beacon):
  refs_cfg = beacon.get("refs_test")
  m0 = beacon.get("m0")
  s1hg = beacon.get("s1hg")
  s1vo = beacon.get("s1vo")

  assert refs_cfg['scan']['axis'].__repr__() == repr(m0)
  assert refs_cfg['slits'][0]['axis'].__repr__() == repr(s1hg)
  assert refs_cfg['slits'][0]['position'] == 0
  assert refs_cfg['slits'][1]['axis'].__repr__() == repr(s1vo)
  assert refs_cfg['slits'][1]['position'] == 1
  assert refs_cfg['m0'].__repr__() == repr(m0)


