__author__ = 'jim'

import glob
import unittest
import logging

def create_test_suite():
    test_file_strings = glob.glob('test/test_*.py')
    module_strings = ['test.'+str[5:len(str)-3] for str in test_file_strings]
    logging.info(module_strings)
    suites = [unittest.defaultTestLoader.loadTestsFromName(name) \
              for name in module_strings]
    logging.info(suites)
    testSuite = unittest.TestSuite(suites)
    return testSuite
