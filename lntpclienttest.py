#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time    : 2019/8/26 19:11
# @Version : 1.0
# @File    : sync_sock.py
# @Author  : Jingsheng Tang
# @Version : 1.0
# @Contact : mrtang@nudt.edu.cn   mrtang_cs@163.com
# @License : (C) All Rights Reserved

from lntp import *

s = LNTPClient(('192.168.166.1',8765))
s.start()

for i in range(10000):
    timestamp,dt,d = s.lntp_clk()
    print(timestamp,end='\r')
    time.sleep(0.01)