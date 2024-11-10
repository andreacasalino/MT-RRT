# import matplotlib.pyplot as plt
import os, json, tempfile, re

def getLogDir():
    if 'MT_RRT_LOG_PATH' in os.environ:
        path = os.path.join(os.environ['MT_RRT_LOG_PATH'], 'benchmark.json')
        with open(path) as stream:
            return stream.read().strip()
    # find something in tmp folder whose name starts with 'MT_RRT_'
    tmpDir = tempfile.gettempdir()
    for path in os.listdir(tmpDir):
        if path.find('MT_RRT_') == 0:
            return os.path.join(tmpDir, path)
    raise Exception('Unable to locate lgo results')
    
class Figure:
    def __init__(self, absPath):
        filename = os.path.basename(absPath)
        m = re.match('^(.*?).benchmark.json$', filename)
        self.title = m.group(1)
        with open(absPath, 'r') as fd:
            self.data = json.load(fd)
        # self.fig, self.ax = plt.subplots()

    def print(self):
        # TODO change me with code to produce real plot
        print('==========================\n{}:\n{}'.format( self.title, json.dumps(self.data, indent=1) ))

if __name__ == '__main__':
    logDir = getLogDir()
    print('===> reading files from: {}'.format(logDir))

    figures = [Figure(os.path.join(logDir, res)) for res in os.listdir(logDir)]
    for fig in figures:
        fig.print()

    # plt.autoscale()
    # plt.show()
