--[[
    示例功能：通过TCP socket与远程服务器进行通信，发送和接收数据
    主要步骤包括：
      1. 建立TCP连接
      2. 发送数据
      3. 循环接收数据并打印到aubo_control日志，直到接收到有效数据
      4. 最后关闭连接
]]
local app = {}
local aubo = require('aubo')
local sched = sched or aubo.sched
local math = aubo.math or math
local realtime = aubo.realtime or realtime

local sleep = sched.sleep
local thread = sched.thread
local sync = sched.sync
local run = sched.run
local kill = sched.kill
local halt = sched.halt

-- 示例函数：通过TCP socket进行发送和接收数据
function example_send_receive_data()
    local _ENV = sched.select_robot(1)
    ip = "172.16.26.248"  -- 目标服务器的IP地址
    port = 8000  -- 目标服务器的端口号
    client_name = "socket_0"  -- 定义客户端名称 
    rec_key = "camera"  -- 接收数据的关键字
    
    -- 关闭当前客户端的socket连接
    socketClose(client_name)
    -- 打开新的socket连接
    socketOpen(ip, port, client_name)
    -- 发送初始数据 "abcd" 到目标服务器
    socketSendLine("abcd", client_name)
    sleep(1)
    rec_str = ""  -- 接收字符串
    
    -- 循环等待直到接收到有效的数据
    while(rec_str == nil or rec_str == "")
    do
        -- 从目标服务器读取数据，并通过指定的关键字存储
        socketReadString(rec_key, client_name, "", "", false)
        sleep(0.1)
        -- 获取接收到的字符串
        rec_str = getString(rec_key, "")
        textmsg("**********")
        textmsg(rec_str)
        textmsg("**********")
        sleep(1)
        -- 发送数据到目标服务器
        socketSendLine("dfgh", client_name)
    end
    -- 在数据接收完成后关闭socket连接
    socketClose(client_name)
end

function app:start(api)
  -- 调用发送接收数据的示例函数
  example_send_receive_data()
end

return app

