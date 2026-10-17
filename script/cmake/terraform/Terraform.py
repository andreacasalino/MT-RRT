import json, os, argparse, tempfile, platform
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

CMakeInfo = namedtuple('CMakeInfo', 'path name cmake_name cmake_parent_name context kind')

def openWithEnvExpansion(path):
    with open(path, 'r') as stream:
        json_str = stream.read()
        # expand env
        for name, val in Env.env.items():
            json_str = json_str.replace('${' + name + '}', val)
            json_str = json_str.replace('$' + name, val)
    return json.loads(json_str)

def getCMakeInfo(path):
    name = os.path.basename(path)
    cmake_name = '{}{}'.format(Env.PREFIX, name)
    context_file = os.path.join(path, 'terraform.json')
    context = openWithEnvExpansion(context_file) if os.path.exists(context_file) else dict()

    def manipulate_name(suffix):
        info_parent = getCMakeInfo(os.path.dirname(path))
        res_name = "{}-{}".format(info_parent.name, suffix)
        res_cmake_name = "{}-{}".format(info_parent.cmake_name, suffix)
        return info_parent, res_name, res_cmake_name

    if name == 'test':
        info_parent, name, cmake_name = manipulate_name(Env.TEST_SUFFIX)
        return CMakeInfo(path, name, cmake_name, info_parent.cmake_name, context, TargetKind.TEST)
    elif name == 'samples':
        
        info_parent, name, cmake_name = manipulate_name(Env.SAMPLE_SUFFIX)
        return CMakeInfo(path, name, cmake_name, info_parent.cmake_name, context, TargetKind.SAMPLE)
    else:
        return CMakeInfo(path, name, cmake_name, None, context, TargetKind.LIBRARY)

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
        self.decorators = [(sourceWithSubstitution("Header", {"THE_SCRIPT": os.path.abspath(__file__)}), "")]
        self.content = ""
        return self

    def __exit__(self, exc_type_, exc_val_, exc_tb_):
        dest = os.path.join(self.info.path, "CMakeLists.txt")
        print('===> generating at {}'.format(dest))
        with open(dest, 'w') as fd:
            fd.write('\n'.join(a for (a, _) in self.decorators))
            fd.write(self.content)
            fd.write('\n'.join(b for (_, b) in self.decorators))
        return True

    def append(self, to_add):
        self.content += f"\n{to_add}"

    def appendMultiple(self, iterable):
        for it in iterable:
            self.append(it)

    def addCondition(self, expression):
        self.decorators.append((f"if({expression})", "endif()"))

def genRequirements(target: CMakeFile, content):
    target.appendMultiple('find_package({} REQUIRED)'.format(dep) for dep in content)

def genDependencies(target: CMakeFile, content):
    for scope, dep in content.items():
        dep_content = sourceWithSubstitution("Dependencies", {
            "THE_NAME": target.info.cmake_name,
            "THE_SCOPE": scope,
            "THE_DEPS": '\n'.join("   {}".format(d) for d in  dep),
        })
        target.append(dep_content)

def genDefinitions(target: CMakeFile, content):
    for scope, defs in content.items():
        for definition in defs:
            frmt = {
                "THE_NAME": target.info.cmake_name,
                "THE_SCOPE": scope,
                "THE_DEFINITION_NAME": definition['name'],
                "THE_DEFINITION_VALUE": definition['value'],
            }
            def_content = sourceWithSubstitution("Definitions", frmt)
            target.append(def_content)
            if definition["export"]:
                def_export_content = sourceWithSubstitution("Definitions-export", frmt)
                target.append(def_export_content)

def genFlags(target: CMakeFile, content):
    for scope, flags in content.items():
        for flag in flags:
            flag_content = sourceWithSubstitution("Flags", {
                "THE_NAME": target.info.cmake_name,
                "THE_SCOPE": scope,
                "THE_OPTION_NAME": '{}{}'.format(Env.PREFIX, flag['name']),
                "THE_DESCRIPTION": flag['msg'],
                "THE_DEFAULT_VALUE": 'ON' if flag['default'] else 'OFF',
            })
            target.append(flag_content)

def genShowCmd(target: CMakeFile, content):
    for cmd in content:
        diag_content = sourceWithSubstitution("Show", {
            'TARGET_TEST': target.info.cmake_name,
            'TEST_CASE_NAME': cmd['name'],
            'DIAG_SCRIPT': cmd['script'],
            'PLATFORM': platform.system(),
            'ARGS_TEST': cmd['args'] if 'args' in cmd else '',
            'ARGS_SCRIPT': ' '.join(cmd['args_script']) if 'args_script' in cmd else ''
        })
        target.append(diag_content)

def generateTargetContent(target: CMakeFile):
    if not target.info.context:
        return

    GRAMMAR = {
        'require':genRequirements,
        'depends':genDependencies,
        'definitions':genDefinitions,
        'flags':genFlags,
        'show':genShowCmd
    }

    for k, v in target.info.context.items():               
        if not k in GRAMMAR:
            continue
        GRAMMAR[k](target, v)

    if 'condition' in target.info.context:
        target.addCondition(' OR '.join(target.info.context['condition']))
        
    if 'optional' in target.info.context:
        content = sourceWithSubstitution("OptionalBuild", {
            "THE_NAME": target.info.cmake_name,
            "DEFAULT_VALUE": 'ON' if target.info.context['optional'] else 'OFF'
        })
        target.decorators.append((content, "endif()"))

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

class Test:
    def __init__(self, target: CMakeFile):
        self.target = target

    def __enter__(self):
        self.target.append("if(BUILD_MT_RRT_TESTS)")
        content = sourceWithSubstitution("Test", {
            "THE_NAME": self.target.info.cmake_name,
            "THE_PARENT_NAME": self.target.info.cmake_parent_name
        })
        self.target.append(content)

    def __exit__(self, exc_type_, exc_val_, exc_tb_):
        self.target.append("endif()")
        # Returning False or None will let exceptions propagate normally.
        return False

class Sample:
    def __init__(self, target: CMakeFile):
        self.target = target
        
    def __enter__(self):
        self.target.append("if(BUILD_MT_RRT_SAMPLES)")
        content = sourceWithSubstitution("Sample", {
            "THE_NAME": self.target.info.cmake_name,
            "THE_PARENT_NAME": self.target.info.cmake_parent_name
        })
        self.target.append(content)
        genShowCmd(self.target, self.samplesGenerator_(self.target))

    def __exit__(self, exc_type_, exc_val_, exc_tb_):
        self.target.append("endif()")
        # Returning False or None will let exceptions propagate normally.
        return False
    
    def locateShowScript_(self):
        fldr = os.path.dirname(self.target.info.path)
        for filename in os.listdir(fldr):
            if not filename.find('.py') == -1:
                return os.path.join(fldr, filename)
        msg = 'Visualization script not located for {}'.format(self.target.info.cmake_name)
        raise Exception(msg)

    def samplesGenerator_(self):
        show_script = self.locateShowScript_(self.target)
        for sample_json in filter(lambda name: not name.find('.json') == -1, os.listdir(self.target.info.path)):
            sample_json_abs = os.path.join(self.target.info.path, sample_json) 
            sample_json_name_without_ext = sample_json.split('.')[0]
            yield {
                'name': "{}-{}".format(self.target.info.name, sample_json_name_without_ext),
                'script': show_script,
                'args': "'{}'".format(sample_json_abs)
            }

def generateTarget(path):
    with CMakeFile(path) as target:
        if target.info.kind == TargetKind.LIBRARY:
            obj_kind = Library
        elif target.info.kind == TargetKind.TEST:
            obj_kind = Test
        elif target.info.kind == TargetKind.SAMPLE:
            obj_kind = Sample
                            
        with obj_kind(target) as _:
            generateTargetContent(target)
             
def generateTargets(path):
    path = os.path.abspath(path)
    subpaths = []
    for sub_path in os.listdir(path):
        sub_path_tot = os.path.join(path, sub_path)
        if not os.path.isdir(sub_path_tot):
            continue
        generateTarget(sub_path_tot)
        subpaths.append(sub_path)
    with open(os.path.join(path, "CMakeLists.txt"), 'w') as fd:
        fd.write('\n'.join("add_subdirectory({})".format(subpath) for subpath in subpaths))

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
