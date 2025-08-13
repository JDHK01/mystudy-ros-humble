### 打包容器，备份镜像

```bash
docker commit ubuntu22.04 ubuntu22.04-backup
```

### 映射本地存储到镜像, 同时设置端口号(2222->22)

```bash
docker run -d \
  --name ubuntu22.04 \
  -p 2222:22 \
  -v /Users/yqz/ubuntu22.04:/workspace \
  --restart unless-stopped \
  ubuntu22.04-backup:latest \
  tail -f /dev/null
```

  -v /Users/yqz/ubuntu22.04:/workspace \

将本地的`/Users/yqz/ubuntu22.04`映射为`/workspace`

PS:pycharm好像不能实现远程的连接, 配置失败



### 迁移到mac上运行

能运行，，但是比较慢.

而且不知道为什么，每次source必须先进入`install`目录，部分内容有冲突

```bash
cd install
source local_setup.bash
```



### mac的环境配置方案

##### 虚拟环境创建

```bash
conda create -n humble python=3.10.12
```

##### 配置源

```bash
# 添加必要的频道
conda config --env --add channels conda-forge
conda config --env --add channels robostack-staging
conda config --env --set channel_priority strict
```

##### 安装

```bash
# 安装核心包
conda install ros-humble-ros-core

# 或者安装桌面版（包含更多工具和可视化）
conda install ros-humble-desktop

# 如果上面的命令失败，可以尝试指定更少的包
conda install ros-humble-ros-base
```

每一个都很慢

##### 配置环境变量和自动补全

```bash
source $CONDA_PREFIX/setup.bash
```

但是这里存在一个问题, 如果你把它写入到`.zshrc`, 每次开启终端都会尝试加载.

但是，它在`humble`的虚拟环境中, 为了防止报错污染终端，更改为

```bash
# humble相关
if [ -n "$CONDA_PREFIX" ] && [ -f "$CONDA_PREFIX/setup.bash" ]; then
    source $CONDA_PREFIX/setup.bash
fi
```

自动补全同理

```bash
# 自动补全
ROS2_COMPLETE="/opt/anaconda3/envs/humble/share/ros2cli/environment/ros2-argcom$
if [ -f "$ROS2_COMPLETE" ]; then
    source $ROS2_COMPLETE
fi
```

##### 配置编译工具

```bash
conda install colcon-common-extensions
```

##### 其他

安装完毕后，会出现有一个包不兼容的情况，卸载后重新安装即可

这里为了节约时间，使用了pip

```bash
pip uninstall cryptography -y
pip install cryptography
```


