import json, os, argparse, platform, tempfile
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

def getCMakeInfo(path):
    res = CMakeInfo(path)
    res.path = path
    res.name = os.path.basename(path)
    res.cmake_name = '{}{}'.format(Env.PREFIX, res.name)

    context_file = os.path.join(res.path, 'terraform.json')
    res.context = openWithEnvExpansion(context_file) if os.path.exists(context_file) else None

    if res.name == 'test':
        res.name = "{}-{}".format(res.name, Env.TEST_SUFFIX)
        res.kind = TargetKind.TEST
    elif res.name == 'sample':
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

def openWithEnvExpansion(path):
    with open(path, 'r') as stream:
        json_str = stream.read()
        # expand env
        for name, val in Env.env.items():
            json_str = json_str.replace('${' + name + '}', val)
            json_str = json_str.replace('$' + name, val)
    return json.loads(json_str)

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

    def write(self):
        dest = os.path.join(self.path, "CMakeLists.txt")
        with open(dest, 'w') as fd:
            fd.write(self.content)

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
        dep_content = sourceWithSubstitution("Dependencies", {
            "THE_SCOPE": scope,
            "THE_NAME": target.info.cmake_name,
            "THE_DEFINITION": dep['name'],
            "THE_VALUE": dep['value'],
        })
        target.append(dep_content)

def genFlags(recipe, info : Info):
    defs = ''
    for scope, definitions in recipe.items():
        for definition in definitions:
            frmt = {
                'CMAKE_NAME':info.cmake_name,
                'SCOPE':scope,
                'OPT_NAME':definition['name'],
                'NAME':'{}{}'.format(Env.PREFIX, definition['name']),
                'MSG':definition['msg'],
                'DEFAULT': 'ON' if definition['default'] else 'OFF'
            }
            defs += """
option({NAME} "{MSG}" {DEFAULT})
if({NAME})
target_compile_definitions({CMAKE_NAME} {SCOPE} {OPT_NAME})
endif()""".format(**frmt)  
    return defs

GRAMMAR = {
    'require':genRequirements,
    'depends':genDependencies,
    'definitions':genDefinitions,
    'flags':genFlags,
    # 'show':genShowCmd
}

def generateTargetContent(target: CMakeFile):
    for k, v in target.info.context.items():                
        if not k in GRAMMAR:
            continue
        GRAMMAR[k](target, v)

    if target.info.context:
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
        tests_path = os.path.join(self.target.info.path, "test")
        if os.path.exists(tests_path):
            self.target.append("add_subdirectory(test)")
            generateTarget(tests_path)

        samples_path = os.path.join(self.target.info.path, "samples")
        if os.path.exists(samples_path):
            self.target.append("add_subdirectory(samples)")
            generateTarget(tests_path)

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
    with open("CMakeLists.txt") as fd:
        for sub_path in os.listdir(path):
            fd.write("add_subdirectory({})".format(sub_path))
            generateTarget(os.path.join(path, sub_path))

# class Test(Target):
#     def setUp_(self):
#         self.initContext_('condition')
#         self.context['condition'].append('BUILD_MT_RRT_TESTS')

#         self.initContext_('depends', 'PUBLIC')
#         for dep in ['GTest::gtest', 'GTest::gtest_main', self.info_parent.cmake_name]:
#             self.context['depends']['PUBLIC'].append(dep)

#         self.initContext_('definitions', 'PUBLIC')
#         for n, v in [('TEST_TAG', '[{}]'.format(self.info_parent.name)), ('TEST_FOLDER', '${CMAKE_CURRENT_SOURCE_DIR}')]:
#             self.context['definitions']['PUBLIC'].append({
#                 'name':n,
#                 'value':v
#             })

#     def __init__(self, info:Info):
#         Target.__init__(self, pathJoin(info.path, 'test'))
#         self.info_parent = info
#         self.info.cmake_name = '{}{}'.format(self.info_parent.cmake_name, Env.TEST_SUFFIX)
#         self.setUp_()

#     def genCore(self):
#         frmt = {
#             'PATH':self.info.path,
#             'NAME':self.info.cmake_name,
#             'SOURCES':'{SOURCES}'
#         }
#         return """
# file(GLOB_RECURSE SOURCES {PATH}/*h {PATH}/*cpp)
# add_executable({NAME} ${SOURCES})
# install(TARGETS {NAME})""".format(**frmt)

# class Sample(Target):
#     def locateScript_(self):
#         fldr = os.path.dirname(self.info.path)
#         for filename in os.listdir(fldr):
#             if not filename.find('.py') == -1:
#                 return pathJoin(fldr, filename)
#         msg = 'Visualization script not located for {}'.format(self.info.cmake_name)
#         raise Exception(msg)

#     def setUp_(self):
#         self.initContext_('depends', 'PUBLIC')
#         self.context['depends']['PUBLIC'].append(self.info_parent.cmake_name)

#         script = self.locateScript_()
#         self.context['show'] = [{
#             'name':filename.split('.')[0],
#             'script':script,
#             'args':"'{}'".format(pathJoin(self.info.path, filename))
#         } for filename in filter(lambda name: not name.find('.json') == -1, os.listdir(self.info.path))]

#     def __init__(self, info:Info):
#         Target.__init__(self, pathJoin(info.path, 'samples'))
#         self.info_parent = info
#         self.info.cmake_name = '{}{}'.format(self.info_parent.cmake_name, Env.SAMPLE_SUFFIX)
#         self.setUp_()

#     def genCore(self):
#         return """
# add_executable({} Main.cpp)""".format(self.info.cmake_name)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-m', '--multiple', default=None)
    parser.add_argument('-s', '--single', default=None)
    args = parser.parse_args()

    if not args.multiple == None:
        path = os.path.abspath(args.single)
        generateTargets(path)

    elif not args.single == None:
        generateTarget(args.single)

#############################################################################################################################

# def getEnvVars():
#     return {
#         'PYTHONPATH':'${COMMON_KIT_PATH}',
#         'MT_RRT_LOG_PATH':'${MT_RRT_LOG_PATH}'
#     }

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
