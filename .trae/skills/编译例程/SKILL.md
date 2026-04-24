---
name: 编译例程
description: 用户主动提及编译例程时
---

使用./waf list查找编译目标，找到以elf结尾的目标
使用./waf build --target 目标名称进行编译，记得在命令行中配置环境变量HPM_SDK_BASE和gnu工具链的路径