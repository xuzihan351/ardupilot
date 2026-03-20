# encoding: utf-8

# flake8: noqa

"""
Waf tool for HPMicro build
"""

from waflib import Build, ConfigSet, Configure, Context, Task, Utils
from waflib import Errors, Logs
from waflib.TaskGen import before, after_method, before_method, feature
from waflib.Configure import conf
from collections import OrderedDict

import os
import shutil
import sys
import traceback
import re
import pickle
import subprocess
import importlib.util

sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../libraries/AP_HAL_HPMICRO/hwdef/scripts'))
import hpmicro_hwdef  # noqa:501

@feature('hpmicro_ap_library', 'hpmicro_ap_program')
@before_method('process_source')
def hpmicro_dynamic_env(self):
    if self.bld.cmd == 'list':
        return

def load_env_vars(env):
    '''optionally load extra environment variables from env.py in the build directory'''
    print("Checking for env.py")
    env_py = os.path.join(env.BUILDROOT, 'env.py')
    if not os.path.exists(env_py):
        print("No env.py found")
        return
    e = pickle.load(open(env_py, 'rb'))
    for k in e.keys():
        v = e[k]
        if k == 'ROMFS_FILES':
            env.ROMFS_FILES += v
            continue
        if k in env:
            if isinstance(env[k], dict):
                a = v.split('=')
                env[k][a[0]] = '='.join(a[1:])
                print("env updated %s=%s" % (k, v))
            elif isinstance(env[k], list):
                env[k].append(v)
                print("env appended %s=%s" % (k, v))
            else:
                env[k] = v
                print("env added %s=%s" % (k, v))
        else:
            env[k] = v
            print("env set %s=%s" % (k, v))

def setup_canmgr_build(cfg):
    '''enable CANManager build. By doing this here we can auto-enable CAN in
    the build based on the presence of CAN pins in hwdef.dat except for AP_Periph builds'''
    env = cfg.env
    env.INCLUDES += [
        cfg.srcnode.find_dir('modules/DroneCAN/libcanard').abspath(),
        cfg.srcnode.find_dir('libraries/AP_DroneCAN/canard').abspath(),
        ]

    if cfg.options.disable_DroneCAN == 'True':
        env.CFLAGS += ['-DCANARD_ENABLE_CANFD=0']
    else:
        env.CFLAGS += ['-DCANARD_ENABLE_CANFD=1']

    cfg.get_board().with_can = True

def configure(cfg):
    cfg.load('compiler_c')
    cfg.load('asm')
    cfg.load('gcc gas')
    cfg.load('cmake')

    cfg.env.AP_PROGRAM_AS_STLIB = True

    target = "hpmicro"
    bldnode = cfg.bldnode.make_node(cfg.variant)
    def srcpath(path):
        return cfg.srcnode.make_node(path).abspath()
    def bldpath(path):
        return bldnode.make_node(path).abspath()

    #define env and location for the cmake hpmicro file
    env = cfg.env
    env.DEFINES+=['USE_NONVECTOR_MODE=1', 'DISABLE_IRQ_PREEMPTIVE=1', 'CONFIG_FREERTOS=1']
    env.AP_PROGRAM_FEATURES += ['hpmicro_ap_program']
    
    env.CFLAGS+=['-mabi=ilp32d', '-march=rv32imafdc_zicsr_zifencei'];
    env.CXXFLAGS+=['-mabi=ilp32d', '-march=rv32imafdc_zicsr_zifencei'];

    env.BUILDROOT = bldpath('')
    env.SRCROOT = srcpath('')
    env.APJ_TOOL = srcpath('Tools/scripts/apj_tool.py')

    try:
        generate_hwdef_h(env)
    except Exception as e:
        print(get_exception_stacktrace(e))
        cfg.fatal("Failed to generate hwdef")
    load_env_vars(cfg.env)
    as_prog = cfg.env.TOOLCHAIN + '-gcc'
    cfg.env.AS = as_prog
    cfg.env.COMPILER_AS = as_prog

    if env.HAL_NUM_CAN_IFACES and not env.AP_PERIPH:
        setup_canmgr_build(cfg)
    if not env.DEBUG:
        env.CUSTOM_GCC_LINKER_FILE = 'flash_xip_ardupilot'
    else:
        env.CUSTOM_GCC_LINKER_FILE = 'flash_xip_ardupilot_debug'
def get_exception_stacktrace(e):
    ret = "%s\n" % e
    ret += ''.join(traceback.format_exception(type(e),
                                              e,
                                              tb=e.__traceback__))
    return ret

def generate_hwdef_h(env):
    '''run hpmicro_hwdef.py'''
    hwdef_dir = os.path.join(env.SRCROOT, 'libraries/AP_HAL_HPMICRO/hwdef')

    if len(env.HWDEF) == 0:
        env.HWDEF = os.path.join(hwdef_dir, env.BOARD, 'hwdef.dat')
    hwdef_out = env.BUILDROOT
    if not os.path.exists(hwdef_out):
        os.mkdir(hwdef_out)
    hwdef = [env.HWDEF]
    if env.HWDEF_EXTRA:
        hwdef.append(env.HWDEF_EXTRA)
    eh = hpmicro_hwdef.HPMicroHWDef(
        outdir=hwdef_out,
        hwdef=hwdef,
        quiet=False,
    )
    eh.run()

def pre_build(self):

    from waflib import Task

    load_env_vars(self.env)
    lib_vars = OrderedDict()
    lib_vars['ARDUPILOT_CMD'] = self.cmd
    lib_vars['WAF_BUILD_TARGET'] = self.targets
    lib_vars['ARDUPILOT_LIB'] = self.bldnode.find_or_declare('lib/').abspath()
    lib_vars['ARDUPILOT_BIN'] = self.bldnode.find_or_declare('lib/bin').abspath()

    hwdef_h = os.path.join(self.env.BUILDROOT, 'hwdef.h')
    if not os.path.exists(hwdef_h):
        print("Generating hwdef.h")
        try:
            generate_hwdef_h(self.env)
        except Exception:
            self.fatal(f"Failed to process hwdef.dat {hwdef_h}")
    # Process includes.list from AP_HAL_HPMICRO targets: substitute {ARDUPILOT_PATH}
    try:
        includes_src = os.path.join(self.env.SRCROOT, 'libraries/AP_HAL_HPMICRO/targets/includes.list')
        if os.path.exists(includes_src):
            with open(includes_src, 'r') as fh:
                inc_data = fh.read()
            # replace placeholder with project source root
            inc_data = inc_data.replace('{ARDUPILOT_PATH}', self.env.SRCROOT)
            inc_data = inc_data.replace('{ARDUPILOT_BOARD_NAME}', self.env.BOARD)
            inc_data = inc_data.replace('{HPM_SDK_BASE}', os.getenv('HPM_SDK_BASE', os.path.join(self.env.SRCROOT, 'modules/hpm_sdk')))
            # write processed includes file into build directory
            out_includes = os.path.join(self.env.BUILDROOT, 'includes_ap_hal_hpmicro.list')
            with open(out_includes, 'w') as fh:
                fh.write(inc_data)
            # expose path to env for later use in build()
            self.env.HPM_AP_HAL_INCLUDES = out_includes
    except Exception:
        # non-fatal: continue with existing behavior
        pass

    self.bldnode.find_or_declare('lib')
    self.bldnode.find_or_declare('lib/bin')

    includes = self.bldnode.find_or_declare(self.env.HPM_AP_HAL_INCLUDES).read().split()
    self.env.prepend_value('INCLUDES', includes)
    includes = self.bldnode.find_or_declare('../../modules/tinyalloc').abspath()
    self.env.prepend_value('INCLUDES', includes)
    includes = self.bldnode.find_or_declare('modules/DroneCAN/libcanard/dsdlc_generated/include').abspath()
    self.env.prepend_value('INCLUDES', includes)

def build(bld):

    # Clean build directory if exists
    build_dir = os.path.join(bld.env.BUILDROOT, 'build')
    if os.path.isdir(build_dir):
        import shutil
        shutil.rmtree(build_dir)

    env_cflags = list(bld.env.CFLAGS)
    if '-Werror=shadow' in env_cflags:
        env_cflags.remove('-Werror=shadow')

@feature('hpmicro_ap_program')
@after_method('post_link')
def hpmicro_firmware(self):
    self.link_task.always_run = True
    lib_vars = OrderedDict()
    lib_vars['BOARD'] = self.env.BOARD
    lib_vars['BOARD_SEARCH_PATH'] = self.bld.srcnode.abspath() + '/libraries/AP_HAL_HPMICRO/boards/boards'
    if not self.env.DEBUG:
        lib_vars['CMAKE_BUILD_TYPE'] = 'Release'
    else:
        lib_vars['CMAKE_BUILD_TYPE'] = 'Debug'
    lib_vars['HPM_BUILD_TYPE'] = 'flash_xip'
    libtext = self.bld.bldnode.find_or_declare('lib/'+self.program_dir + '/lib' + self.program_name + '.a').abspath()
    for u in Utils.to_list(getattr(self, 'use', [])):
        target_u = self.bld.get_tgen_by_name(u).target
        if '/' in target_u:
            dir_part, name_part = os.path.split(target_u)
            lib_path = self.bld.bldnode.find_or_declare(f'lib/{dir_part}/lib{name_part}.a')
        else:
            lib_path = self.bld.bldnode.find_or_declare('lib/lib'+ target_u + '.a')
        libtext += ' ' + lib_path.abspath()
    lib_vars['LIBNAME'] = libtext
    lib_vars['CUSTOM_GCC_LINKER_FILE'] = self.env.CUSTOM_GCC_LINKER_FILE
    lib_vars['HPM_SOC_SERIES'] = self.env.HPM_SOC_SERIES
    cmake_target = self.bld.cmake(
            name=self.program_name,
            cmake_vars=lib_vars,
            cmake_src='libraries/AP_HAL_HPMICRO/cmake',
            cmake_bld='build/'+ self.name,
            )
    build = cmake_target.build('demo.elf', target='demo.elf', path=self.bld.srcnode.find_or_declare('build/'+ self.name+'/output'))
    build.post()
    build.cmake_build_task.set_run_after(self.link_task)

