import json, os, argparse, tempfile
from collections import namedtuple
from enum import Enum

class Env:
    TERRAFORM_ROOT=os.path.dirname(__file__)
    PREFIX='MT-RRT-'
    TEST_SUFFIX='Tests'
    SAMPLE_SUFFIX='Samples'
    env={
        'PREFIX':PREFIX,
        'LOG_PATH': os.path.join(tempfile.gettempdir(), "MT-RRT-LogPath"),
    }

class TargetKind(Enum):
    LIBRARY = 1
    TEST = 2
    SAMPLE = 3

CMakeInfo = namedtuple('CMakeInfo', 'path name cmake_name context kind')

def openWithEnvExpansion(path):
    with open(path, 'r') as stream:
        json_str = stream.read()
        # expand env
        for name, val in Env.env.items():
            json_str = json_str.replace('${' + name + '}', val)
            json_str = json_str.replace('$' + name, val)
    return json.loads(json_str)

def getCMakeInfo(path):
    res = CMakeInfo(path)
    res.path = path
    res.name = os.path.basename(path)
    res.cmake_name = '{}{}'.format(Env.PREFIX, res.name)

    context_file = os.path.join(res.path, 'terraform.json')
    res.context = openWithEnvExpansion(context_file) if os.path.exists(context_file) else dict()

    if res.name == 'test':
        res.name = "{}-{}".format(res.name, Env.TEST_SUFFIX)
        res.kind = TargetKind.TEST
    elif res.name == 'samples':
        res.name = "{}-{}".format(res.name, Env.SAMPLE_SUFFIX)
        res.kind = TargetKind.SAMPLE
    else:
        res.kind = TargetKind.LIBRARY

    return res

def sourceWithSubstitution(filename, frmt):
    path_abs = os.path.join(Env.TERRAFORM_ROOT, f"{filename}.in.cmake")
    with open(path_abs, 'r') as fd:
        content = fd.read()
    for k, v in frmt.items():
        content = content.replace(f"$${k}", v)
    return content

class CMakeFile:
    def __init__(self, path):
        self.info = getCMakeInfo(path)

    def __enter__(self):
        self.content = sourceWithSubstitution("Header", {"THE_SCRIPT": os.path.abspath(__file__)})
        return self

    def __exit__(self, exc_type_, exc_val_, exc_tb_):
        dest = os.path.join(self.path, "CMakeLists.txt")
        print('===> generating at {}'.format(dest))
        with open(dest, 'w') as fd:
            fd.write(self.content)
        # Returning False or None will let exceptions propagate normally.
        return False

    def append(self, to_add):
        self.content += f"\n{to_add}"

    def appendMultiple(self, iterable):
        for it in iterable:
            self.append(it)

    def decorate(self, before, after):
        self.content = "{}\n{}\n{}".format(before, self.content, after)

def genRequirements(target: CMakeFile, content):
    target.appendMultiple('find_package({} REQUIRED)'.format(dep) for dep in content)

def genDependencies(target: CMakeFile, content):
    for scope, dep in content.items():
        dep_content = sourceWithSubstitution("Dependencies", {
            "THE_SCOPE": scope,
            "THE_NAME": target.info.cmake_name,
            "THE_DEPS": '\n'.join(dep),
        })
        target.append(dep_content)

def genDefinitions(target: CMakeFile, content):
    for scope, dep in content.items():
        def_content = sourceWithSubstitution("Definitions", {
            "THE_SCOPE": scope,
            "THE_NAME": target.info.cmake_name,
            "THE_DEFINITION": dep['name'],
            "THE_VALUE": dep['value'],
        })
        target.append(def_content)

def genFlags(target: CMakeFile, content):
    for scope, dep in content.items():
        # TODO check again this one with the previous version
        flag_content = sourceWithSubstitution("Flags", {
            "THE_TARGET": target.info.cmake_name,
            "THE_NAME": '{}{}'.format(Env.PREFIX, dep['name']),
            "THE_DESCRIPTION": dep['msg'],
            "THE_VALUE_DEFAULT": 'ON' if dep['default'] else 'OFF',
            "THE_SCOPE": scope
        })
        target.append(flag_content)

def generateTargetContent(target: CMakeFile):
    if not target.info.context:
        return

    GRAMMAR = {
        'require':genRequirements,
        'depends':genDependencies,
        'definitions':genDefinitions,
        'flags':genFlags,
        # 'show':genShowCmd
    }

    for k, v in target.info.context.items():               
        if not k in GRAMMAR:
            continue
        GRAMMAR[k](target, v)

    if 'condition' in target.info.context:
        before = "if({})".format(' OR '.join(target.info.context['condition']))
        target.decorate(before, "endif()")
        
    if 'optional' in target.info.context:
        before = sourceWithSubstitution("OptionalBuild", {
            "THE_NAME": target.info.cmake_name,
            "DEFAULT_VALUE": 'ON' if target.info.context['optional'] else 'OFF'
        })
        target.decorate(before, "endif()")

class Library:
    def __init__(self, target: CMakeFile):
        self.target = target

    def __enter__(self):
        content = sourceWithSubstitution("Library", {"THE_NAME": self.target.info.cmake_name})
        self.target.append(content)

    def __exit__(self, exc_type_, exc_val_, exc_tb_):
        for subpath in ['test', 'samples']:
            subpath_abs = os.path.join(self.target.info.path, subpath)
            if os.path.exists(subpath_abs):
                self.target.append("add_subdirectory({})".format(subpath))
                generateTarget(subpath_abs)

        # Returning False or None will let exceptions propagate normally.
        return False

def generateTarget(path):
    with CMakeFile(path) as target:
        if target.info.kind == TargetKind.LIBRARY:
            with Library(target) as _:
                generateTargetContent(target)

        else:
            generateTargetContent(target)
             
def generateTargets(path):
    path = os.path.abspath(path)
    with open("CMakeLists.txt", 'w') as fd:
        for sub_path in os.listdir(path):
            fd.write("add_subdirectory({})".format(sub_path))
            generateTarget(os.path.join(path, sub_path))

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-m', '--multiple', default=None)
    parser.add_argument('-s', '--single', default=None)
    args = parser.parse_args()

    if not args.multiple == None:
        path = os.path.abspath(args.multiple)
        generateTargets(path)

    elif not args.single == None:
        generateTarget(args.single)

#############################################################################################################################

# def genShowCmd(recipe, info : Info):
#     cmd = ''
#     for rec in recipe:
#         frmt = {
#             'MT_RRT_ROOT':'${MT_RRT_ROOT}',
#             'PARENT':info.cmake_name,
#             'TARGET_TEST':info.cmake_name,
#             'TEST_CASE_NAME':rec['name'],
#             'PYTHON_CMD':'${PYTHON_CMD}',
#             'DIAG_SCRIPT':rec['script'],
#             'PLATFORM':platform.system(),
#             'ARGS_TEST': rec['args'] if 'args' in rec else '',
#             'ARGS_SCRIPT': ' '.join(rec['args_script']) if 'args_script' in rec else '',
#             'ENV':CommandEnv.env
#         }
#         cmd += """
# MakeTestDiag({TARGET_TEST} {TEST_CASE_NAME} {DIAG_SCRIPT} {PLATFORM} '{ARGS_TEST}' '{ARGS_SCRIPT}')
# """.format(**frmt)
#     return cmd
