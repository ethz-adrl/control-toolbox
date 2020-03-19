#!/usr/bin/env python

from __future__ import print_function

import os
import sys
import subprocess

def main(argv=sys.argv[1:]):
    working_dir = argv[0]
    tests = argv[1:]
    print("Working directory: {}".format(working_dir))
    print("Running tests: ", tests)

    test_dir = working_dir + "/test_results"
    if not os.path.exists(test_dir):
      os.makedirs(test_dir)

    for the_file in os.listdir(test_dir):
      file_path = os.path.join(test_dir, the_file)
      try:
          if os.path.isfile(file_path):
              os.unlink(file_path)
      except Exception as e:
          print(e)

    commands = []

    for cmd in tests:
        test_cmd = "./" + cmd + " --gtest_output=xml:" + working_dir + "/test_results/"
        print("Running: ", test_cmd)
        rc = subprocess.call(test_cmd, cwd=working_dir, shell=True)
        if rc == True : #in case of test failure
          sys.exit(1)

        commands.insert(0, test_cmd)

    print("Ran the following commands: ")
    for cmd in commands:
      print("-- ", cmd)

if __name__ == '__main__':
    sys.exit(main())
