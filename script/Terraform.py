import json, os, argparse, platform

def pathJoin(front, *args):
    res = front
    for piece in args:
        res = os.path.join(res, piece)
    res = os.path.normpath(res)
    res = str(res).replace('\\', '/')
    return res

class Env:
    PREFIX='MT-RRT-'
    TEST_SUFFIX='-Tests'
    SAMPLE_SUFFIX='-Samples'
    env={
        'PREFIX':PREFIX
    }

    @staticmethod
    def expandEnv(json_str):     
        res = json_str
        for name, val in Env.env.items():
            res = res.replace('${' + name + '}', val)
            res = res.replace('$' + name, val)
        return res

class Info:
    def __init__(self, path):
        self.path = path
        self.name = os.path.basename(path)
        self.cmake_name = '{}{}'.format(Env.PREFIX, self.name)

def genRequirements(recipe, info : Info):
    return '\n'.join('find_package({} REQUIRED)'.format(dep) for dep in recipe)

def genDependencies(recipe, info : Info):
    res = ''
    for scope, deps in recipe.items():
        res += """
{}
{}""".format(scope, '\n'.join(deps))    
    return """
target_link_libraries({} 
{}
)""".format(info.cmake_name, res)

def genDefinitions(recipe, info : Info):
    defs = ''
    for scope, definitions  in recipe.items():
        for definition in definitions:
            frmt = {
                'CMAKE_NAME':info.cmake_name,
                'SCOPE':scope,
                'NAME':definition['name'],
                'VALUE':definition['value']
            }
            defs += """
target_compile_definitions({CMAKE_NAME} {SCOPE}
-D {NAME}="{VALUE}"
)""".format(**frmt) 
            if 'export' in definition and definition['export']:
                defs += """\nSET({NAME} "{VALUE}" CACHE INTERNAL "{NAME}")""".format(**frmt)
    return defs

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

def getEnvVars():
    return {
        'PYTHONPATH':'${COMMON_KIT_PATH}',
        'MT_RRT_LOG_PATH':'${MT_RRT_LOG_PATH}'
    }

class CommandEnv:
    @staticmethod
    def make():
        frmt = {
            'Linux':"""export {}={}""",
            'Windows':"""set {}={}"""
        }
        frmt_to_use = 'COMMAND {}'.format(frmt[platform.system()])
        return '\n'.join(frmt_to_use.format(k, v) for k, v in getEnvVars().items())

    env = make()

def genShowCmd(recipe, info : Info):
    cmd = ''
    for rec in recipe:
        frmt = {
            'MT_RRT_ROOT':'${MT_RRT_ROOT}',
            'PARENT':info.cmake_name,
            'NAME':'{}-{}'.format(info.cmake_name, rec['name']),
            'BIN_PATH': '${' + '{}-bin-path'.format(info.cmake_name) + '}',
            'ARGS': rec['args'] if 'args' in rec else '',
            'ARGS_SCRIPT': ' '.join(rec['args_script']) if 'args_script' in rec else '',
            'PYTHON_CMD':'${PYTHON_CMD}',
            'SCRIPT':rec['script'],
            'ENV':CommandEnv.env
        }
        pltfrm = platform.system()
        if pltfrm == 'Windows':
            frmt['ENV'] = ' '.join(['{}={}'.format(k, v) for k, v, in getEnvVars().items()])
            cmd += """            
add_executable({NAME} {MT_RRT_ROOT}/launcher/Main.cpp)
target_compile_definitions({NAME} PUBLIC -D ENV="{ENV}")
target_compile_definitions({NAME} PUBLIC -D BIN_PATH="{BIN_PATH}")
target_compile_definitions({NAME} PUBLIC -D ARGS="{ARGS}")
target_compile_definitions({NAME} PUBLIC -D PYTHON_CMD="{PYTHON_CMD}")
target_compile_definitions({NAME} PUBLIC -D SCRIPT="{SCRIPT}")
target_compile_definitions({NAME} PUBLIC -D ARGS_SCRIPT="{ARGS_SCRIPT}")
add_dependencies({NAME} {PARENT})
""".format(**frmt)
        else:
            cmd += """
add_custom_target({NAME} 
DEPENDS {PARENT}
COMMAND {BIN_PATH} {ARGS}
{ENV}
COMMAND {PYTHON_CMD} {SCRIPT} {ARGS_SCRIPT}
SOURCES {SCRIPT}
)""".format(**frmt)
    return cmd

BLOCKS = {
    'require':genRequirements,
    'depends':genDependencies,
    'definitions':genDefinitions,
    'flags':genFlags,
    'show':genShowCmd
}

class Target:
    def __init__(self, path):
        self.info = Info(path)
        self.context = {}
        context_file = pathJoin(self.info.path, 'terraform.json')
        if os.path.exists(context_file):
            with open(context_file, 'r') as stream:
                json_str = stream.read()
                json_str = Env.expandEnv(json_str)
                self.context = json.loads(json_str)            

    def initContext_(self, name, scope=None):
        if not name in self.context:
            self.context[name] = [] if scope == None else {}
        if not scope == None and not scope in self.context[name]:
            self.context[name][scope] = []
    
    def decorateWithConditions(self, content):
        if 'condition' in self.context:
            cond = set(self.context['condition'])
            content = """
if({})
{}
endif()
            """.format(' OR '.join(cond) , content) 
        return content
                    
    def decorateWithOptional(self, content):
        if 'optional' in self.context:
            frmt = {
                'OPT_NAME':'BUILD_{}'.format(self.info.cmake_name),
                'NAME':self.info.name,
                'DEFAULT_VALUE':'ON' if self.context['optional'] else 'OFF',
                'CONTENT':content
            }
            content = """
option({OPT_NAME} "Build {NAME}" {DEFAULT_VALUE})
if({OPT_NAME})
{CONTENT}
endif()
            """.format(**frmt)
        return content

    def genExportBinPath_(self):
        frmt = {
            'TARGET':self.info.cmake_name,
            'CMAKE_CURRENT_BINARY_DIR':'${CMAKE_CURRENT_BINARY_DIR}'
        }
        return """
if (CMAKE_GENERATOR MATCHES "Visual Studio")
    SET({TARGET}-bin-path  "{CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>/{TARGET}" CACHE INTERNAL "{TARGET}-bin-path")
else()
    SET({TARGET}-bin-path  "{CMAKE_CURRENT_BINARY_DIR}/{TARGET}" CACHE INTERNAL "{TARGET}-bin-path")
endif()
""".format(**frmt)

    def gen(self):
        with open(pathJoin(self.info.path, 'CMakeLists.txt'), 'w') as stream:
            stream.write("""###########################################################################
##### This file was autogenerated by {}
###########################################################################\n\n""".format(__file__))
            content = self.genCore()
            content += '\n{}'.format(self.genExportBinPath_())
            for topic, recipe in self.context.items():                
                if not topic in BLOCKS:
                    continue
                content += '\n{}'.format(BLOCKS[topic](recipe, self.info))
            content = self.decorateWithConditions(content)
            content = self.decorateWithOptional(content)
            stream.write(content)

class Test(Target):
    def setUp_(self):
        self.initContext_('condition')
        self.context['condition'].append('BUILD_MT_RRT_TESTS')

        self.initContext_('depends', 'PUBLIC')
        for dep in ['GTest::gtest', 'GTest::gtest_main', self.info_parent.cmake_name]:
            self.context['depends']['PUBLIC'].append(dep)

        self.initContext_('definitions', 'PUBLIC')
        for n, v in [('TEST_TAG', '[{}]'.format(self.info_parent.name)), ('TEST_FOLDER', '${CMAKE_CURRENT_SOURCE_DIR}')]:
            self.context['definitions']['PUBLIC'].append({
                'name':n,
                'value':v
            })

    def __init__(self, info:Info):
        Target.__init__(self, pathJoin(info.path, 'test'))
        self.info_parent = info
        self.info.cmake_name = '{}{}'.format(self.info_parent.cmake_name, Env.TEST_SUFFIX)
        self.setUp_()

    def genCore(self):
        frmt = {
            'PATH':self.info.path,
            'NAME':self.info.cmake_name,
            'SOURCES':'{SOURCES}'
        }
        return """
file(GLOB_RECURSE SOURCES {PATH}/*h {PATH}/*cpp)
add_executable({NAME} ${SOURCES})
install(TARGETS {NAME})""".format(**frmt)

class Sample(Target):
    def locateScript_(self):
        fldr = os.path.dirname(self.info.path)
        for filename in os.listdir(fldr):
            if not filename.find('.py') == -1:
                return pathJoin(fldr, filename)
        msg = 'Visualization script not located for {}'.format(self.info.cmake_name)
        raise Exception(msg)

    def setUp_(self):
        self.initContext_('depends', 'PUBLIC')
        self.context['depends']['PUBLIC'].append(self.info_parent.cmake_name)

        script = self.locateScript_()
        self.context['show'] = [{
            'name':filename.split('.')[0],
            'script':script,
            'args':"'{}'".format(pathJoin(self.info.path, filename))
        } for filename in filter(lambda name: not name.find('.json') == -1, os.listdir(self.info.path))]

    def __init__(self, info:Info):
        Target.__init__(self, pathJoin(info.path, 'samples'))
        self.info_parent = info
        self.info.cmake_name = '{}{}'.format(self.info_parent.cmake_name, Env.SAMPLE_SUFFIX)
        self.setUp_()

    def genCore(self):
        return """
add_executable({} Main.cpp)""".format(self.info.cmake_name)

class Library(Target):
    SPECIAL_FOLDERS = {
        'test':Test,
        'samples':Sample
    }

    def genCore(self):
        frmt = {
            'PATH':self.info.path,
            'NAME':self.info.cmake_name,
            'SOURCES':'{SOURCES}',
        }
        content = """
file(GLOB_RECURSE SOURCES {PATH}/header/* {PATH}/src/*)

if(LIB_OPT)
    if (WIN32)
        set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
    endif ()	
    
    add_library({NAME} SHARED ${SOURCES})
else()
    add_library({NAME} STATIC ${SOURCES})
endif()

target_include_directories({NAME} PUBLIC 
    {PATH}/header
)

install(TARGETS {NAME})
install(DIRECTORY {PATH}/header/ DESTINATION include/{NAME} FILES_MATCHING PATTERN "*.h*")""".format(**frmt)

        for name, type_name in Library.SPECIAL_FOLDERS.items():
            if os.path.exists(pathJoin(self.info.path, name)):
                type_name(self.info).gen()
                content += '\nadd_subdirectory({})\n'.format(name)
        
        return content

class CMake:
    def __init__(self, root):
        self.root = root

    def gen(self):
        with open(pathJoin(self.root, 'CMakeLists.txt'), 'w') as stream:
            for subpath in filter(lambda p: os.path.isdir(pathJoin(self.root, p)), os.listdir(self.root)):
                path = pathJoin(self.root, subpath)
                print('===> generating at {}'.format(path))
                lib = Library(path)
                lib.gen()
                stream.write('add_subdirectory({})\n'.format(subpath))

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-m', '--multiple', default=None)
    parser.add_argument('-s', '--single', default=None)
    args = parser.parse_args()

    if not args.multiple == None:
        handler = CMake(args.multiple)
        handler.gen()
    elif not args.single == None:
        print('===> generating at {}'.format(args.single))
        handler = Library(args.single)
        handler.gen()
