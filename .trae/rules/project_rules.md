./waf是编译工具
HPM_SDK_BASE=/home/ubuntu/ardupilot_verify/ardupilot3/ardupilot/modules/hpm_sdk/是需要配置的环境变量
GNURISCV_TOOLCHAIN_PATH=/home/ubuntu/riscv32-gnu-toolchain是需要配置的环境变量
./waf 支持clean distclean configure copter命令
./waf list 是查看所有板子的命令
我在开发的soc平台是hpmicro，板子是hpmpilotbf67
./waf configure 是配置板子的命令
我在开发的目标是copter，./waf copter是编译copter的命令
编译过程是waf工具将ardupilot中的代码编译成若干个库文件，然后调用cmake产生hpm_sdk的工程，ardupilot的库文件通过变量的形式传递给hpm_sdk工程，最后调用make或者ninja编译hpm_sdk的工程
执行./waf copter时，需要在命令行中配置环境变量HPM_SDK_BASE和GNURISCV_TOOLCHAIN_PATH
build/{board_name}/build是hpm_sdk工程编译输出目录
build/{board_name}是ardupilot工程编译输出目录
./waf命令不需要在沙盒中运行
