# Start
## 安装docker
Windows 下需要安装 WSL2 和 Docker Desktop，以及图形化界面支持的VcXsrv
Linux 下直接安装 Docker 即可（当然也需要配置图形化界面）


## 构建镜像


Windows下使用指令    
```bash
cd docker/build/
docker build --network host -t pathplanner_ws:kinetic -f .\ros_workspace.dockerfile .
```

Linux下使用指令(区分文件路径的写法)
``` bash
cd docker/build
docker build --network host -t pathplanner_ws:kinetic -f ros_workspace.dockerfile .
```

## 运行容器
运行docker/scripts/下的脚本
根据平台进行选择
./docker/scripts/Win/run.sh

## 进入容器

./docker/scripts/Win/into.sh

## 快捷操作
可以配置vscode的task，使用快捷键操作，在.vscode下创建task.json文件    
参考以下  
```json
{
  "version": "2.0.0",
  "tasks": [
    {
      "label": "exec-into-container",
      "type": "shell",
      "command": "./docker/scripts/Win/into.sh",
      "options": {
        "cwd": "${workspaceFolder}",
      },
    },
    {
      "label": "run-container",
      "type": "shell",
      "command": "./docker/scripts/Win/run.sh",
      "options": {
        "cwd": "${workspaceFolder}",
      }
    }
  ]
}
```

# 注意事项
如果是要进行一些包的安装，建议在dockerfile末尾添加命令，以便环境同步更新  
否则这些更改在你关闭容器再开启时会丢失  

