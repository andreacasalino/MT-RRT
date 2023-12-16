import subprocess, argparse, os

class TestMap:
    def forEachProcess(self):
        for name in filter(lambda name: not name.find('Tests') == -1, os.listdir(self.folder)):
            yield name

    def __init__(self, folder=None):
        self.folder = os.getcwd() if folder == None else folder

    def __str__(self):
        return '\n'.join(self.forEachProcess())

    def run(self):
        failed = []
        for name in self.forEachProcess():
            print('>>>===========================================================<<<')
            print('running {}'.format(name))
            print('>>>===========================================================<<<')
            completed = subprocess.run(['./{}'.format(name)], cwd=self.folder, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            if not completed.returncode == 0:
                failed.append(name)
            print('{}\n{}\n'.format(completed.stdout, completed.stderr))
        if not len(failed) == 0:
            msg = 'These tests failed\n{}'.format('\n'.join(failed))
            raise Exception(msg)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--folder', default=None)
    parser.add_argument('--list', action='store_true', default=False)
    parser.add_argument('--run', action='store_true', default=False)
    args = parser.parse_args()

    map = TestMap(args.folder)

    if args.list:
        print(map)
    elif args.run:
        map.run()

if __name__ == '__main__':
    main()
