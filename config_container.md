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