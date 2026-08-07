

使用方式：

在当前目录打开cmd， 执行命令：   SplitIvwRes.exe packed_res.bin

   提示: 这个 packed_res.bin 就是三合一的资源, 也就是从 aiui 平台上打包出来的浅定制合并资源(3.17.7).
        aiui平台上打包出来的 3.17.7 资源和 3.17.12 引擎是兼容的,  3.17.12是3.17.7的升级版本, 
		这个3.17.7版本资源可以用此工具解包后放到VTN3中搭配3.17.12的唤醒引擎使用.


示例： SplitIvwRes.exe xiaofeixiaofei_deep_317.bin

执行成功后会在当前目录下生成3个bin文件，分别是 mlp/filler/keyword.bin,
   这3个资源就可以放到vtn3对应文件夹(比如你起个文件夹名字是 abcd , 那么资源一般是放置到 res/ivw_3.17.12/abcd 文件夹下)供引擎使用.
   注意记得修改 vtn.ini 或 代码中的 res_identifier 值为 abcd 来指向你配置的这个唤醒资源.


