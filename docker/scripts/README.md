# 启动容器
./badminton_controls_run.sh
# 进入容器
./badminton_controls_into.sh

# 注意事项
如果是要进行一些包的安装，建议在dockerfile末尾添加命令，以便环境同步更新  
否则这些更改在你关闭容器再开启时会丢失  

