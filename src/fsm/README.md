参考
https://blog.csdn.net/liaoshengg/article/details/81014770
https://blog.csdn.net/sinat_35761779/article/details/52366507
https://blog.csdn.net/larry_zeng1/article/details/80346653
https://blog.csdn.net/u013052238/article/details/80394895
https://blog.csdn.net/xufeng0991/article/details/40746473
https://blog.csdn.net/u013052238/article/details/80394895
https://blog.csdn.net/DFSAE/article/details/52995034
https://blog.csdn.net/xiaojuliet/article/details/46776375

协程
https://blog.csdn.net/wuhenyouyuyouyu/article/details/48262447
https://blog.csdn.net/DFSAE/article/details/53019037?utm_source=blogxgwz2


明天修改Exception中//需要确定是任务完成还是状态恢复
任务状态返回上一次状态逻辑错误，应该修改为返回standby，然后在回到未完成任务状态，但是不发送任务命令，