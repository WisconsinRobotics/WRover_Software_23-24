#!/usr/bin/env python

import json
import os
import shutil
import subprocess
import sys
import time

############
# CONSTANTS
############

# borrowed from blender src
COL_HEADER = '\033[95m'
COL_OKBLUE = '\033[94m'
COL_OKGREEN = '\033[92m'
COL_WARN = '\033[93m'
COL_FATAL = '\033[31m'
COL_CLEAR = '\033[0m'

############
# FUNCTIONS
############

def print_usage():
    print COL_FATAL + 'Usage:'
    for action_name in BUILD_ACTIONS:
        print BUILD_ACTIONS[action_name].get_help()
    print COL_CLEAR

def fork_cmd(args, cwd, fatal_errors=True, relay_io=True):
    stdout = None
    stderr = None
    if not relay_io:
        devnull = open(os.devnull, 'w')
        stdout = devnull
        stderr = devnull
    exit_code = subprocess.Popen(args, cwd=cwd, stdout=stdout, stderr=stderr).wait()
    if fatal_errors and exit_code != 0:
        print '\n' + COL_FATAL + 'FATAL: Command exited with ' + str(exit_code) + ': ' + ' '.join(args)
        print 'Working dir: ' + cwd + COL_CLEAR
        sys.exit(1)

def print_section(text):
    print COL_OKBLUE + text

def print_arrow(col, text):
    print col + '> ' + COL_CLEAR + text

def print_subsec(text):
    print_arrow(COL_OKGREEN, text)

def print_warn(text):
    print COL_WARN + '[?] ' + text

########
# TYPES
########

class Dependency:
    def __init__(self, name, repo, build):
        self.name = name
        self.repo = repo
        self.build = build
    
    def get_repo_dir(self, root_dir):
        return root_dir + '/src/' + self.name

# build preconditions

class Precondition(object):
    def check(self, manifest, dependencies, root_dir, args):
        raise NotImplementedError()

class PreconditionInitialized(Precondition):
    def check(self, manifest, dependencies, root_dir, args):
        if not os.path.isdir(root_dir + '/venv'):
            raise ValueError('Venv does not exist! Have you initialized?')
        if not os.path.isdir(root_dir + '/src'):
            raise ValueError('Source dir does not exist! Have you initialized?')
        if not os.path.isdir(root_dir + '/devel'):
            raise ValueError('Devel space does not exist! Have you initialized?')

class PreconditionSourced(Precondition):
    def check(self, manifest, dependencies, root_dir, args):
        env_var_ros = os.environ.get('ROS_PACKAGE_PATH')
        if not (env_var_ros and (root_dir + '/src') in env_var_ros):
            raise ValueError('Correct ROS source path not found! Have you sourced your workspace?')
        env_var_venv = os.environ.get('VIRTUAL_ENV')
        if not (env_var_venv and (root_dir + '/venv') in env_var_venv):
            raise ValueError('Venv not found! Have you activated the venv?')

# build actions

class BuildAction(object):
    def __init__(self):
        self.name = self.__class__.__name__[11:].lower()

    def dispatch(self, manifest, dependencies, root_dir, args):
        raise NotImplementedError()

    def get_help(self):
        raise NotImplementedError()

    def get_preconditions(self):
        return []

class BuildActionInit(BuildAction):
    def dispatch(self, manifest, dependencies, root_dir, args):
        noop = True

        # init venv
        if not os.path.isdir(root_dir + '/venv'):
            print_section('Initializing venv...' + COL_CLEAR)
            fork_cmd(['virtualenv', '--system-site-packages', '--no-setuptools', '--python=/usr/bin/python2', 'venv'], root_dir)
            print
            noop = False

        # init catkin workspace
        if not os.path.isdir(root_dir + '/devel'):
            print_section('Initializing catkin workspace...' + COL_CLEAR)
            try:
                os.mkdir(root_dir + '/src')
            except:
                pass
            fork_cmd(['catkin_make', '--make-args', 'dontbuildme'], root_dir, fatal_errors=False, relay_io=False)
            print
            noop = False

        # print message if nothing happened
        if noop:
            print_section('No initialization required; doing nothing')
            print

    def get_help(self):
        return 'assemble.py init'

class BuildActionBuild(BuildAction):
    def dispatch(self, manifest, dependencies, root_dir, args):
        # should we operate offline?
        offline_mode = '-o' in args
        if offline_mode:
            print_warn('Offline mode enabled; remote repos will not be queried!\n')

        # should we force dep repos to be redownloaded?
        force_clone = '-f' in args
        if force_clone:
            if offline_mode:
                raise ValueError('Force mode is not supported in offline mode!')
            print_warn('Force mode enabled; all deps will be redownloaded!\n')

        for dependency in dependencies:
            print_section('Processing dependency: ' + dependency.name)
            dep_dir = dependency.get_repo_dir(root_dir)

            # clone repo if necessary
            if dependency.repo:
                if not os.path.isdir(dep_dir):
                    if offline_mode:
                        raise ValueError('No local copy found! Could not build in offline mode.')
                    print_arrow(COL_WARN, 'No local copy found; cloning...')
                    fork_cmd(['git', 'clone', dependency.repo, dep_dir], root_dir)
                elif not offline_mode:
                    if force_clone:
                        print_warn('> Redownload forced; cloning...' + COL_CLEAR)
                        shutil.rmtree(dep_dir, ignore_errors=True)
                        fork_cmd(['git', 'clone', dependency.repo, dep_dir], root_dir)
                    else:
                        print_subsec('Pulling repo...')
                        fork_cmd(['git', 'pull'], dep_dir)

            # perform build
            dependency.build.execute(root_dir, dep_dir, args)
            print

        # check for project-level requirements.txt
        py_req_file = root_dir + '/requirements.txt'
        if os.path.isfile(py_req_file):
            print_section('Installing project-level Python dependencies...' + COL_CLEAR)
            pip_args = ['pip', 'install']
            if 'pip_args' in manifest:
                for arg in manifest['pip_args']:
                    pip_args.append(arg)
            pip_args.append('-r')
            pip_args.append('requirements.txt')
            fork_cmd(pip_args, root_dir)
            print


        # invoke catkin
        print_section('Delegating remainder of build to catkin...\n' + COL_CLEAR)
        fork_cmd(['catkin_make'], root_dir)
        print

    def get_help(self):
        return 'assemble.py build [-o] [-f]'

    def get_preconditions(self):
        return [PreconditionInitialized(), PreconditionSourced()]

class BuildActionClean(BuildAction):
    def dispatch(self, manifest, dependencies, root_dir, args):
        for dependency in dependencies:
            print_section('Cleaning ' + dependency.name)
            print_subsec('Purging local repo...')
            shutil.rmtree(dependency.get_repo_dir(root_dir), ignore_errors=True)

            print_subsec('Purging generated files...')
            for path in dependency.build.get_generated(root_dir):
                try:
                    os.remove(path)
                except:
                    pass
            print

    def get_help(self):
        return 'assemble.py clean'

    def get_preconditions(self):
        return [PreconditionInitialized()]

# build providers

class BuildProvider(object):
    def create_build(self, root_dir, dto):
        raise NotImplementedError()

class Build(object):
    def execute(self, root_dir, dep_dir, args):
        raise NotImplementedError()

    def get_generated(self, root_dir):
        return []

class BuildProviderCMake(BuildProvider):
    def create_build(self, root_dir, dto):
        build_dto = dto['build']

        # construct build dir
        build_dir = root_dir + '/src/' + dto['name']
        if 'build_dir' in build_dto:
            build_dir += '/' + build_dto['build_dir']

        # gather cmake args
        cmake_args = ['cmake']
        if 'cmake_args' in build_dto:
            for arg in build_dto['cmake_args']:
                cmake_args.append(arg)
        cmake_args.append('.')

        # gather make args
        make_args = ['make']
        if 'make_args' in build_dto:
            for arg in build_dto['make_args']:
                make_args.append(arg)

        return BuildCMake(build_dir, build_dto['artifacts'], build_dto['dependents'], cmake_args, make_args)


class BuildCMake(Build):
    def __init__(self, build_dir, artifacts, dependents, cmake_args, make_args):
        self.build_dir = build_dir
        self.artifacts = artifacts
        self.dependents = dependents
        self.cmake_args = cmake_args
        self.make_args = make_args

    def execute(self, root_dir, dep_dir, args):
        # configure build with cmake
        print_subsec('Invoking ' + ' '.join(self.cmake_args))
        fork_cmd(self.cmake_args, cwd=self.build_dir)

        # perform build with make
        print_subsec('Invoking ' + ' '.join(self.make_args))
        fork_cmd(self.make_args, cwd=self.build_dir)

        # attempt to copy lib to each dependent
        for dest in self.dependents:
            print_subsec('Copying to dependent: ' + dest)
            dest_dir = root_dir + '/src/' + dest + '/lib' # destination dir for dependent
            try:
                os.makedirs(dest_dir)
            except:
                pass
            for artifact in self.artifacts:
                shutil.copy2(dep_dir + '/' + artifact, dest_dir)

    def get_generated(self, root_dir):
        generated = []
        for dest in self.dependents:
            for artifact in self.artifacts:
                generated.append(root_dir + '/src/' + dest + '/lib/' + os.path.basename(artifact))
        return generated

class BuildProviderPip(BuildProvider):
    def create_build(self, root_dir, dto):
        pip_args = ['pip', 'install']
        if 'pip_args' in dto['build']:
            for arg in dto['build']['pip_args']:
                pip_args.append(arg)
        pip_args.append('-r')
        pip_args.append('requirements.txt')
        return BuildPip(pip_args)

class BuildPip(Build):
    def __init__(self, pip_args):
        self.pip_args = pip_args

    def execute(self, root_dir, dep_dir, args):
        req_file = dep_dir + '/requirements.txt'
        if os.path.isfile(req_file):
            print_subsec('Invoking ' + ' '.join(self.pip_args))
            fork_cmd(self.pip_args, dep_dir)
        else:
            print_subsec('No pip dependencies; doing nothing')

class BuildProviderApt(BuildProvider):
    install_list = None

    def create_build(self, root_dir, dto):
        if 'package' in dto['build']:
            return BuildApt(dto['build']['package'])
        else:
            return BuildApt(dto['name'])

class BuildApt(Build):
    def __init__(self, pkg_name):
        self.pkg_name = pkg_name

    def execute(self, root_dir, dep_dir, args):
        if not BuildProviderApt.install_list:
            print_subsec('Caching dpkg install list...')
            BuildProviderApt.install_list = set()
            with open('/var/lib/dpkg/status') as dpkg_status:
                for line in dpkg_status:
                    if line.startswith('Package: '):
                        BuildProviderApt.install_list.add(line[9:].strip())
        if self.pkg_name in BuildProviderApt.install_list:
            print_subsec('Ignoring already-installed package: ' + self.pkg_name)
        else:
            print_subsec('Installing APT package: ' + self.pkg_name)
            fork_cmd(['sudo', 'apt', 'install', '-y', self.pkg_name], root_dir)
            BuildProviderApt.install_list.add(self.pkg_name)

class BuildProviderNoop(BuildProvider):
    def create_build(self, root_dir, dto):
        return BuildNoop()

class BuildNoop(Build):
    def execute(self, root_dir, dep_dir, args):
        print_subsec('No-op build; doing nothing')

#########
# TABLES
#########

BUILD_ACTIONS = {
    'init': BuildActionInit(),
    'build': BuildActionBuild(),
    'clean': BuildActionClean()
}

BUILD_PROVIDERS = {
    'cmake': BuildProviderCMake(),
    'pip': BuildProviderPip(),
    'apt': BuildProviderApt(),
    'noop': BuildProviderNoop()
}

##############
# ENTRY POINT
##############

def main(args):
    # ensure we have at least a build action
    if not args:
        print_usage()
        sys.exit(1)
    
    # retrieve action
    action = BUILD_ACTIONS.get(args[0])
    if action is None:
        print_usage()
        sys.exit(1)
    action_args = args[1:]

    # find project root dir
    root_dir = os.path.dirname(os.path.realpath(__file__))
    print COL_HEADER + 'Project root: ' + COL_CLEAR + root_dir
    print # python 2 sucks

    # load project manifest
    dependencies = []
    manifest = None
    with open('project.json', 'r') as project_file:
        manifest = json.load(project_file)
        print COL_OKBLUE + 'Found ' + str(len(manifest['deps'])) + ' dependencies:' + COL_CLEAR
        for dto in manifest['deps']:
            print '- ' + dto['name']
            build_provider = BUILD_PROVIDERS.get(dto['build']['provider'])
            if build_provider is None:
                print COL_FATAL + 'Unknown build provider: ' + dto['build']['provider'] + COL_CLEAR
                sys.exit(1)
            dependencies.append(Dependency(dto['name'], dto.get('repo'), build_provider.create_build(root_dir, dto)))
    print

    # check preconditions
    try:
        for precondition in action.get_preconditions():
            precondition.check(manifest, dependencies, root_dir, action_args)
    except ValueError, e:
        print COL_FATAL + 'Build precondition not met: ' + e.args[0] + COL_CLEAR
        sys.exit(1)

    # dispatch action
    runtime = time.time()
    print COL_HEADER + 'Dispatching action: ' + COL_CLEAR + action.name
    print
    action.dispatch(manifest, dependencies, root_dir, action_args)
    runtime = time.time() - runtime
    print COL_HEADER + 'Finished in ' + COL_CLEAR + str(int(runtime * 1000)) + COL_HEADER + ' ms' + COL_CLEAR

if __name__ == '__main__':
    try:
        main(sys.argv[1:])
    except Exception:
        print COL_FATAL + 'Build Failed!' + COL_CLEAR
        raise
