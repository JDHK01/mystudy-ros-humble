# 配置桌面-VNC

### 环境配置

```bash
#!/bin/bash
echo "=== 系统环境准备 ==="

# 更新软件包列表
apt-get update

# 安装基础工具
apt-get install -y \
  wget \
  curl \
  vim \
  net-tools \
  software-properties-common \
  apt-transport-https \
  ca-certificates \
  gnupg \
  lsb-release
```

```bash
# 设置用户环境变量
export USER=root
export HOME=/root
export LANG=C.UTF-8
export LANGUAGE=en_US:en
export LC_ALL=C.UTF-8

# 写入bashrc
cat >> ~/.bashrc << 'EOF'
export USER=root
export HOME=/root
export LANG=C.UTF-8
export LANGUAGE=en_US:en
export LC_ALL=C.UTF-8
EOF

# 重新加载环境变量
source ~/.bashrc
```

```bash
echo "=== 安装x11vnc和虚拟显示 ==="

# 安装x11vnc和Xvfb
apt-get install -y \
  x11vnc \
  xvfb \
  xserver-xorg-core \
  xserver-xorg-input-all \
  xserver-xorg-video-dummy

# 安装websockify和noVNC（Web访问）
apt-get install -y \
  websockify \
  novnc

# 安装字体支持
apt-get install -y \
  fonts-dejavu \
  fonts-liberation \
  fonts-ubuntu \
  fonts-noto \
  fontconfig
```

```bash
# 浏览器三选一
# 选项1：安装Firefox (snap版本)
apt-get install -y snapd
snap install firefox

# 选项2：安装Chromium
apt-get install -y chromium-browser

# 选项3：安装轻量级浏览器
apt-get install -y midori
```

```bash
echo "=== 安装中文支持 ==="

apt-get install -y \
  language-pack-zh-hans \
  fonts-wqy-zenhei \
  fonts-wqy-microhei \
  ibus \
  ibus-pinyin

# 生成中文语言环境
locale-gen zh_CN.UTF-8
```

### 配置X11

```bash
mkdir -p /etc/x11vnc
mkdir -p ~/.vnc
mkdir -p /var/log/x11vnc
```

```bash
cat > /etc/x11vnc/xvfb.conf << 'EOF'
# Xvfb配置文件
DISPLAY=:1
SCREEN=0
RESOLUTION=1280x720
DEPTH=24
DPI=96
EOF
```

```bash
cat > /etc/x11vnc/x11vnc.conf << 'EOF'
# x11vnc配置文件
display :1
rfbport 5901
shared
forever
nopw
listen localhost
noxdamage
noxfixes
noxrandr
wait 20
defer 10
threads
cursor arrow
EOF
```

### 写脚本

##### 启动

```bash
cat > /usr/local/bin/start-vnc-desktop << 'EOF'
#!/bin/bash

##############################################
#         x11vnc VNC桌面环境启动脚本
##############################################

# 配置变量
DISPLAY_NUM=1
VNC_PORT=5901
WEB_PORT=6080
RESOLUTION=1280x720
DEPTH=24

# 日志文件
LOG_DIR="/var/log/x11vnc"
XVFB_LOG="$LOG_DIR/xvfb.log"
X11VNC_LOG="$LOG_DIR/x11vnc.log"
DESKTOP_LOG="$LOG_DIR/desktop.log"
WEBSOCKIFY_LOG="$LOG_DIR/websockify.log"

# 创建日志目录
mkdir -p "$LOG_DIR"

# 设置环境变量
export USER=root
export HOME=/root
export DISPLAY=:$DISPLAY_NUM
export XAUTHORITY="$HOME/.Xauth"

echo "================================================"
echo "       启动x11vnc VNC桌面环境"
echo "================================================"
echo "显示: $DISPLAY"
echo "分辨率: $RESOLUTION"
echo "VNC端口: $VNC_PORT"
echo "Web端口: $WEB_PORT"
echo "================================================"

# 函数：清理现有进程
cleanup_processes() {
    echo "1. 清理现有进程..."
    
    # 停止websockify
    pkill -f websockify >/dev/null 2>&1
    
    # 停止x11vnc
    pkill -f x11vnc >/dev/null 2>&1
    
    # 停止桌面环境
    pkill -f xfce4-session >/dev/null 2>&1
    pkill -f startxfce4 >/dev/null 2>&1
    
    # 停止Xvfb
    pkill -f "Xvfb :$DISPLAY_NUM" >/dev/null 2>&1
    
    # 清理锁文件和套接字
    rm -f /tmp/.X$DISPLAY_NUM-lock
    rm -f /tmp/.X11-unix/X$DISPLAY_NUM
    
    # 等待进程完全退出
    sleep 2
    
    echo "   ✓ 进程清理完成"
}

# 函数：启动虚拟显示服务器
start_xvfb() {
    echo "2. 启动虚拟显示服务器(Xvfb)..."
    
    # 启动Xvfb
    Xvfb :$DISPLAY_NUM \
        -screen 0 ${RESOLUTION}x${DEPTH} \
        -ac \
        -nolisten tcp \
        -dpi 96 \
        +extension GLX \
        +render \
        -noreset \
        > "$XVFB_LOG" 2>&1 &
    
    # 等待Xvfb启动
    local timeout=10
    local count=0
    while [ $count -lt $timeout ]; do
        if xdpyinfo -display :$DISPLAY_NUM >/dev/null 2>&1; then
            echo "   ✓ Xvfb启动成功 (显示 :$DISPLAY_NUM)"
            return 0
        fi
        sleep 1
        count=$((count + 1))
    done
    
    echo "   ✗ Xvfb启动失败"
    return 1
}

# 函数：启动桌面环境
start_desktop() {
    echo "3. 启动XFCE桌面环境..."
    
    # 等待X服务器完全准备好
    sleep 2
    
    # 设置X认证
    touch "$XAUTHORITY"
    xauth add "$DISPLAY" . "$(xxd -l 16 -p /dev/urandom)"
    
    # 启动桌面环境
    startxfce4 > "$DESKTOP_LOG" 2>&1 &
    
    # 等待桌面环境启动
    sleep 5
    
    # 检查桌面是否启动
    if pgrep -f xfce4-session >/dev/null; then
        echo "   ✓ XFCE桌面环境启动成功"
        return 0
    else
        echo "   ✗ XFCE桌面环境启动失败"
        return 1
    fi
}

# 函数：启动x11vnc
start_x11vnc() {
    echo "4. 启动x11vnc服务器..."
    
    x11vnc \
        -display :$DISPLAY_NUM \
        -rfbport $VNC_PORT \
        -shared \
        -forever \
        -nopw \
        -listen localhost \
        -noxdamage \
        -noxfixes \
        -noxrandr \
        -wait 20 \
        -defer 10 \
        -threads \
        -cursor arrow \
        -bg \
        -o "$X11VNC_LOG"
    
    # 等待x11vnc启动
    sleep 3
    
    # 检查x11vnc是否启动
    if netstat -tlnp | grep -q ":$VNC_PORT "; then
        echo "   ✓ x11vnc服务器启动成功 (端口 $VNC_PORT)"
        return 0
    else
        echo "   ✗ x11vnc服务器启动失败"
        return 1
    fi
}

# 函数：启动websockify
start_websockify() {
    echo "5. 启动websockify (Web访问代理)..."
    
    websockify \
        --web=/usr/share/novnc/ \
        $WEB_PORT \
        localhost:$VNC_PORT \
        > "$WEBSOCKIFY_LOG" 2>&1 &
    
    # 等待websockify启动
    sleep 2
    
    # 检查websockify是否启动
    if netstat -tlnp | grep -q ":$WEB_PORT "; then
        echo "   ✓ websockify启动成功 (端口 $WEB_PORT)"
        return 0
    else
        echo "   ✗ websockify启动失败"
        return 1
    fi
}

# 函数：显示状态信息
show_status() {
    echo ""
    echo "================================================"
    echo "          VNC桌面环境启动完成！"
    echo "================================================"
    echo ""
    echo "📋 连接信息："
    echo "   🌐 Web访问:     http://localhost:$WEB_PORT"
    echo "   🖥️  VNC客户端:  localhost:$VNC_PORT"
    echo "   🔐 VNC密码:     无需密码"
    echo ""
    echo "📊 服务状态："
    netstat -tlnp | grep -E ":($VNC_PORT|$WEB_PORT) " | while read line; do
        echo "   $line"
    done
    echo ""
    echo "📝 日志文件："
    echo "   Xvfb日志:      $XVFB_LOG"
    echo "   x11vnc日志:    $X11VNC_LOG"
    echo "   桌面日志:      $DESKTOP_LOG"
    echo "   websockify日志: $WEBSOCKIFY_LOG"
    echo ""
    echo "📋 运行的进程："
    ps aux | grep -E "(Xvfb|x11vnc|websockify|xfce)" | grep -v grep | while read line; do
        echo "   $line"
    done
    echo ""
    echo "💡 提示："
    echo "   - 在浏览器中打开 http://localhost:$WEB_PORT"
    echo "   - 点击 'Connect' 即可进入桌面"
    echo "   - 使用 stop-vnc-desktop 命令停止服务"
    echo ""
}

# 主执行流程
main() {
    # 执行启动步骤
    cleanup_processes || exit 1
    start_xvfb || exit 1
    start_desktop || exit 1
    start_x11vnc || exit 1
    start_websockify || exit 1
    
    # 显示状态
    show_status
    
    echo "✅ VNC桌面环境启动成功！"
}

# 执行主函数
main "$@"
EOF

# 设置执行权限
chmod +x /usr/local/bin/start-vnc-desktop
```

##### 暂停

```bash
cat > /usr/local/bin/stop-vnc-desktop << 'EOF'
#!/bin/bash

echo "================================================"
echo "         停止x11vnc VNC桌面环境"
echo "================================================"

# 停止websockify
echo "1. 停止websockify..."
pkill -f websockify
echo "   ✓ websockify已停止"

# 停止x11vnc
echo "2. 停止x11vnc..."
pkill -f x11vnc
echo "   ✓ x11vnc已停止"

# 停止桌面环境
echo "3. 停止桌面环境..."
pkill -f xfce4-session
pkill -f startxfce4
echo "   ✓ 桌面环境已停止"

# 停止Xvfb
echo "4. 停止Xvfb..."
pkill -f "Xvfb :1"
echo "   ✓ Xvfb已停止"

# 清理锁文件
echo "5. 清理临时文件..."
rm -f /tmp/.X1-lock
rm -f /tmp/.X11-unix/X1
echo "   ✓ 临时文件已清理"

echo ""
echo "✅ VNC桌面环境已完全停止！"
EOF

chmod +x /usr/local/bin/stop-vnc-desktop
```

##### 重启

```bash
cat > /usr/local/bin/restart-vnc-desktop << 'EOF'
#!/bin/bash

echo "重启VNC桌面环境..."
stop-vnc-desktop
sleep 3
start-vnc-desktop
EOF

chmod +x /usr/local/bin/restart-vnc-desktop
```

##### 状态检查

```bash
cat > /usr/local/bin/status-vnc-desktop << 'EOF'
#!/bin/bash

echo "================================================"
echo "         VNC桌面环境状态检查"
echo "================================================"

# 检查Xvfb
if pgrep -f "Xvfb :1" >/dev/null; then
    echo "✓ Xvfb (虚拟显示):     运行中"
else
    echo "✗ Xvfb (虚拟显示):     未运行"
fi

# 检查x11vnc
if pgrep -f x11vnc >/dev/null; then
    echo "✓ x11vnc (VNC服务器):  运行中"
else
    echo "✗ x11vnc (VNC服务器):  未运行"
fi

# 检查桌面环境
if pgrep -f xfce4-session >/dev/null; then
    echo "✓ XFCE (桌面环境):     运行中"
else
    echo "✗ XFCE (桌面环境):     未运行"
fi

# 检查websockify
if pgrep -f websockify >/dev/null; then
    echo "✓ websockify (Web代理): 运行中"
else
    echo "✗ websockify (Web代理): 未运行"
fi

echo ""
echo "📊 端口监听状态："
netstat -tlnp | grep -E ":(5901|6080) " || echo "   没有发现VNC相关端口"

echo ""
echo "📋 相关进程："
ps aux | grep -E "(Xvfb|x11vnc|websockify|xfce)" | grep -v grep || echo "   没有发现相关进程"
EOF

chmod +x /usr/local/bin/status-vnc-desktop
```















```bash
apt install ros-humble-desktop -y# 1. 清理 apt 缓存
apt clean && apt autoclean
rm -rf /var/lib/apt/lists/*

# 2. 更换为阿里云镜像源 (ARM64)
cat > /etc/apt/sources.list << 'EOF'
deb http://mirrors.aliyun.com/ubuntu-ports/ jammy main restricted universe multiverse
deb http://mirrors.aliyun.com/ubuntu-ports/ jammy-updates main restricted universe multiverse
deb http://mirrors.aliyun.com/ubuntu-ports/ jammy-backports main restricted universe multiverse
deb http://mirrors.aliyun.com/ubuntu-ports/ jammy-security main restricted universe multiverse
EOF

# 3. 更新软件包列表
apt update

# 4. 修复损坏的安装
apt --fix-broken install -y

# 5. 重新尝试安装 ROS 2
apt install ros-humble-desktop -y# 1. 清理 apt 缓存
apt clean && apt autoclean
rm -rf /var/lib/apt/lists/*

# 2. 更换为阿里云镜像源 (ARM64)
cat > /etc/apt/sources.list << 'EOF'
deb http://mirrors.aliyun.com/ubuntu-ports/ jammy main restricted universe multiverse
deb http://mirrors.aliyun.com/ubuntu-ports/ jammy-updates main restricted universe multiverse
deb http://mirrors.aliyun.com/ubuntu-ports/ jammy-backports main restricted universe multiverse
deb http://mirrors.aliyun.com/ubuntu-ports/ jammy-security main restricted universe multiverse
EOF

# 3. 更新软件包列表
apt update

# 4. 修复损坏的安装
apt --fix-broken install -y

# 5. 重新尝试安装 ROS 2
apt install ros-humble-desktop -y

```