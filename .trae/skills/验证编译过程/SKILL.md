---
name: 验证编译过程
description: 验证编译的各个环节是否符合要求
---

触发技能时，需要先清空build目录，保持干净的工作空间
然后列出支持的开发板列表，判断是否存在支持的hpmicro开发板
对于每个支持的开发板，使用configure命令配置工程
然后验证copter是否能编译成功，elf和bin文件是否正确生成
然后验证clean命令是否能够把ardupilot和hpm_sdk的中间文件清除