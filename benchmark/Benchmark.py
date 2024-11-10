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

class Label:
    KINDS = {
        '0':'Empty',
        '1':'SmallObstacle',
        '2':'Cluttered'
    }
    STRATEGIES = {
        '0':'Single',
        '1':'Star',
    }

    @staticmethod
    def parse(name):
        args = name.split('-')
        return Label.KINDS[args[1]], Label.STRATEGIES[args[2]], int(args[3]), int(args[4]) if len(args) == 5 else 1

class Samples:
    def __init__(self, label):
        self.label = label
        self.data = {}

    def add(self, threads, samples):
        self.data[threads] = samples

    def show(self, ax):
        self.data = dict(sorted(self.data.items()))
        # TODO
        pass

class Result:
    def __init__(self, title):
        self.title = title
        self.series = {}

    def add(self, iterations, threads, samples):
        if not iterations in self.series:
            self.series[iterations] = Samples()
        self.series[iterations].add(threads, samples)
        
    def show(self):
        print('================')
        # for iters, data in self.series:
        #     print('{} {}')
        # self.fig, self.ax = plt.subplots()
        # TODO for each series show with a unique color
        # TODO legenda
        pass

class Benchmark:
    def __init__(self, absPath):
        filename = os.path.basename(absPath)
        m = re.match('^(.*?).benchmark.json$', filename)
        title = m.group(1)
        with open(absPath, 'r') as fd:
            dataJSON = json.load(fd)
        self.results = {}
        for label, data in dataJSON.items():
            kind, strategy, iters, threads = Label.parse(label)
            name = '{}-{}-{}'.format(title, kind, strategy)
            if not name in self.results:
                self.results[name] = Result(name)
            self.results[name].add(iters, threads, data)

    def show(self):
        for _, result in self.results.items():
            result.show()

if __name__ == '__main__':
    logDir = getLogDir()

    benchamrks = [Benchmark(os.path.join(logDir, res)) for res in os.listdir(logDir)]
    for benchamrk in benchamrks:
        benchamrk.show()

    # plt.autoscale()
    # plt.show()
