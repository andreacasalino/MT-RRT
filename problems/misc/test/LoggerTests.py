import unittest

class LoggerTests(unittest.TestCase):
    def test_canImportCommon(self):
        from common.TestFunction import okIsWorking
        okIsWorking()

    def test_scanLoggerFolder(self):
        import os
        self.assertTrue(not os.environ['MT_RRT_LOG_FOLDER'].find('problems/misc/tmp') == 1)

if __name__ == '__main__':
    unittest.main()
