__author__ = 'jim'

""" run all unit tests assoicated with the IPS system """

# see
# http://www.openp2p.com/pub/a/python/2004/12/02/tdd_pyunit.html#python-s-unittest-module
# http://stackoverflow.com/questions/1896918/running-unittest-with-typical-test-directory-structure

import unittest
import logging

import test.all_tests


logging.basicConfig(format='%(levelname)s:%(asctime)s %(message)s',level=logging.INFO)


testSuite = test.all_tests.create_test_suite()
text_runner = unittest.TextTestRunner().run(testSuite)

# Pause for graphical output to be viewed
# waitKey(0)