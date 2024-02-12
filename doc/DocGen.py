import os, subprocess, shutil

class FileHandler:        
    def __init__(self, fileName=None):
        self.src = fileName
        with open(fileName, 'r') as stream:
            self.contents = [line.strip() for line in stream.readlines()]
        
    def getContent(self):
        return '\n'.join(self.contents)
        
    def reprint(self, fileName = None):
        with open(self.src if fileName == None else fileName, 'w') as stream:
            stream.write('\n'.join(self.contents))
    
    def replace(self, toReplace, toPut):
        self.contents = [line.replace(toReplace, toPut) for line in self.contents]
        return self

    def replaceLine(self, involvedLine, toPut, instances = None):
        for index in self.findLines_(involvedLine, instances):
            self.contents[index] = toPut
        return self
    
    def addBefore(self, involvedLine, toPut, instances = None):
        added = 0
        for index in self.findLines_(involvedLine, instances):
            self.contents.insert(index + added, toPut)
            added += 1
        return self

    def addAfter(self, involvedLine, toPut, instances = None):
        added = 0
        for index in self.findLines_(involvedLine, instances):
            self.contents.insert(index + added + 1, toPut)
            added += 1
        return self
        
    def findLines_(self, line, max_instances = None):
        indices = []
        k = 0
        for content in self.contents:
            if content == line:
                indices.append(k)
                if not max_instances == None and len(indices) == max_instances:
                    break
            k += 1
        return indices

class Paths:
    THIS_FILE_PARENT = os.path.dirname(__file__)
    ROOT = os.path.dirname(THIS_FILE_PARENT)

    @staticmethod
    def make(*args, fromRoot= False):
        res = Paths.ROOT if fromRoot else Paths.THIS_FILE_PARENT
        for piece in args:
            res = os.path.join(res, piece)
        return res

def run(cmd, show = False, cwd = None):
    print('running {}'.format(' '.join(cmd)))
    res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, cwd=cwd, text=True)
    if show:
        print(res.stdout)
    if len(res.stderr) != 0:
        print(res.stderr)
    res.check_returncode()

def forEachSubFolder(parent):
    for name in os.listdir(parent):
        name_abs = os.path.join(parent, name)
        if os.path.isdir(name_abs):
            yield name_abs

class BuildFolder:
    def __init__(self):
        self.root = Paths.make('build')
        shutil.rmtree(self.root, ignore_errors=True) # clean up
        os.makedirs(self.root)
        self.latex = Paths.make('build', 'latex')
        self.html = Paths.make('build', 'html')
        self.doxy_config = os.path.join(self.root, 'doxy_config')
        shutil.copy(Paths.make('doxy_config'), self.doxy_config)
        shutil.copytree(Paths.make('src'), os.path.join(self.root, 'src'))
        src = [os.path.join(folder, 'header', 'MT-RRT') for folder in forEachSubFolder(Paths.make('src', fromRoot=True)) ]
        print('Identified sources:\n{}'.format('\n'.join(src)))
        FileHandler(self.doxy_config).replace('$THE_SOURCES', ' '.join(src)).reprint()

def main():
    build_folder = BuildFolder()

    run(['doxygen', 'doxy_config'], cwd=build_folder.root)

    # modify latex src
    texHandler = FileHandler(os.path.join(build_folder.latex, 'refman.tex'))
    texHandler.addAfter("\\renewcommand{\\numberline}[1]{#1~}" , FileHandler(Paths.make('src/packages.tex')).getContent(), 1)
    texHandler.addAfter("%--- Begin generated contents ---" , FileHandler(Paths.make("src/additional_Sections.tex")).getContent(), 1)
    texHandler.addBefore("\\end{document}", "\\bibliography{../src/ref}{}" + "\n" + "\\bibliographystyle{plain}", 1)
    texHandler.reprint()

    # compile
    run(['pdflatex', 'refman'], cwd=build_folder.latex)
    run(['bibtex', 'refman'], cwd=build_folder.latex)
    for _ in range(0, 3):
        run(['pdflatex', 'refman'], cwd=build_folder.latex)
    shutil.copy(os.path.join(build_folder.latex, 'refman.pdf'), Paths.make('MT-RRT.pdf'))

if __name__ == '__main__':
    main()
