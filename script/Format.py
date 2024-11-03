import subprocess, argparse, os, re, sys
from io import StringIO

class Process:
    def __init__(self, cmd):
        hndlr = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        self.out, self.err = hndlr.communicate()
        if not hndlr.returncode == 0:
            msg = 'something went wrong running: `{}`'.format(cmd)
            raise Exception(msg)

    def stdout(self):
        return self.out

    def stderr(self):
        return self.err
    
    def stdoutStream(self):
        for line in StringIO(self.out).readlines():
            yield line.strip()

    def stderrStream(self):
        for line in StringIO(self.err).readlines():
            yield line.strip()

class ClangFormat(Process):
    try:
        Process('clang-format --version')
    except:
        raise Exception('clang-format was not found on your system!!!')

    def __init__(self, cmd):
        Process.__init__(self, 'clang-format {}'.format(cmd))

def forEachFileInFolder(root):
    for name in os.listdir(root):
        filename = os.path.join(root, name)
        if os.path.isfile(filename):
            yield filename
        else:
            for nested in forEachFileInFolder(filename):
                yield nested

def forEachFile(args):
    if not args.d == None:
        for dir in args.d.split():
            root = os.path.join(os.getcwd(), dir)
            root = os.path.abspath(root)
            for filename in forEachFileInFolder( root ):
                yield filename
    if args.s:
        for name in Process('git show --pretty="" --name-only {}'.format(args.s)).stdoutStream():
            yield os.path.join(os.getcwd() , name.split()[-1])
    if args.k:
        for name in Process('git status -s').stdoutStream():
            yield os.path.join(os.getcwd() , name.split()[-1])

class IsSource:
    accepted = ['h', 'c', 'hxx', 'cxx', 'cc', 'cpp', 'hpp']

    @staticmethod
    def check(filename):
        base = os.path.basename(filename)
        m = re.fullmatch('(.*?)\.(.*?)', base)
        return not m == None and m.groups()[1] in IsSource.accepted

if __name__ == '__main__':
    parser = argparse.ArgumentParser()

    parser.add_argument('-f', action='store_true', help='apply the format')
    parser.add_argument('-c', action='store_true', help='check the format')

    parser.add_argument('-d', default=None, help='directories storing the files to process')
    parser.add_argument('-s', default=None, help='files to process are those contained in the specified SHA')
    parser.add_argument('-k', action='store_true', help='file to process are those changed/staged at the moment')

    args = parser.parse_args()

    if args.f:
        for file in filter(IsSource.check, forEachFile(args)):
            ClangFormat('{} -i'.format(file))

    elif args.c:
        def requiresFormat(filename):
            return not len( ClangFormat( '{} -i -n'.format(filename)).stderr() ) == 0
        unformattedFiles = [file for file in filter(requiresFormat ,filter(IsSource.check, forEachFile(args)))]
        if not len(unformattedFiles) == 0:
            print('These files requires formatting!!\n{}'.format( '\n'.join(unformattedFiles) ))
            sys.exit(1)
