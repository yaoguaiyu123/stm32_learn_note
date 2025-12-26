# 背景

学习过了江科大的stm32标准库

hal库课程：`keysking`

[课程文档](https://docs.keysking.com)

开发板使用的是`正点原子 STM32F407 探索者`

# 环境搭建

## CubeIDE安装

https://docs.keysking.com/docs/stm32/getting-started/getting-started_InstalCube

## 登录 ST 账号

如果没有代理，很难登录，我这里使用代理登录，

- 在windows配置好代理（clash等工具）
- 打开`stm32cubeIDE`

- 菜单栏 **Window** -> **Preferences**

- 搜索 **Network**，找到 **General -> Network Connections**

- 将 `Active Provider` 从 `Native` 改为 **Manual**
- 如果windows早已配置了代理的话，这边会自动检测完成，最后直接点击`apply and close`即可

https://docs.keysking.com/docs/stm32/FAQ/login/#-%E5%A6%82%E4%BD%95%E7%99%BB%E5%BD%95
