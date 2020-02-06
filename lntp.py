#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time    : 2019/8/26 19:11
# @Version : 1.0
# @File    : sync_sock.py
# @Author  : Jingsheng Tang
# @Version : 1.0
# @Contact : mrtang@nudt.edu.cn   mrtang_cs@163.com
# @License : (C) All Rights Reserved

import socket
from precisionclock.clock import clock as getclk
import struct
import time
import numpy as np
import datetime


class LNPTServer():
    def __init__(self,addr):
        lt = time.time()
        gt = getclk()
        self.clk_time_offset = lt - gt

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.myip = addr[0]
        self.sock.bind(addr)
        self.sock.listen(128)

        print('[LNTP] service start!')

    def start(self):
        while True:
            self.client, addr = self.sock.accept()
            print('[LNTP] synchronize network time with: %s-%i'%addr,end='\r')
            if addr[0] == self.myip: #根据ip匹配到来自同一台物理设备
                info = bytes('LNTP-ssfinish',encoding='utf-8')
                offset = struct.pack('d',self.clk_time_offset)
                self.client.send(info + offset)
            else: # 不同设备，开始同步
                info = bytes('LNTP----start', encoding='utf-8')
                self.client.send(info + b'\x00'*8)
                self._syncloop()
            self.client.close()
            print('[LNTP] synchronize network time with client @%s-%i, OK'%addr)

    def _syncloop(self):
        while True:
            buf = self.client.recv(128)
            info = bytes.decode(buf[:13], encoding='utf-8')
            if info == 'LNTP-----over':
                info = bytes('LNTP-s-finish', encoding='utf-8')
                offset = struct.pack('d', self.clk_time_offset)
                self.client.send(info + offset)
                break
            else:
                info = bytes('xxxxxxxxxxxxx', encoding='utf-8')
                clock_buf = struct.pack('d',getclk())
                self.client.send(info + clock_buf)

class LNTPClient():
    def __init__(self,server_addr):
        self.er = None
        self.offset = None
        self.server_addr = server_addr

    def start(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        print('[LNTP] connect to the server', end='\r')
        connect = False

        for i in range(31):
            try:
                self.sock.connect(self.server_addr)
                connect = True
                break
            except socket.error:
                time.sleep(2)

        if not connect:
            raise socket.error('[LNTP] connect to the server: failure')
        else:
            print('[LNTP] connect to the server: OK')

        # 开始同步
        buf = self.sock.recv(128)
        info = bytes.decode(buf[:13], encoding='utf-8')
        if info == 'LNTP-ssfinish':  # 同一物理设备
            self.er = 0
            self.offset = struct.unpack('d', buf[13:21])[0]

        elif info == 'LNTP----start':
            self._sync_loop()
        else:
            raise Exception('unknown error')

        self.sock.close()
        print('[LNTP] sync finish! clock offset: %f'%(self.er+self.offset))

    def _sync_loop(self):
        record = np.zeros((10,4))
        info = bytes('LNTP----doing', encoding='utf-8')
        for i in range(10):
            clkA = getclk()
            clk_buf = struct.pack('d', clkA)
            self.sock.send(info + clk_buf)
            srvbuf = self.sock.recv(128)
            clkB = getclk()
            clk_server = struct.unpack('d',srvbuf[13:21])[0]
            t = 0.5*(clkB - clkA)
            record[i,:] = np.array([t,clkA,clkB,clk_server])
            time.sleep(0.2)

        info = bytes('LNTP-----over--------', encoding='utf-8')
        self.sock.send(info)

        # 取通信时间最短的一组计算参数
        ind = np.argsort(record[:,0])[0]
        t,clkA,clkB,clk_server = record[ind,:]
        self.er = clk_server - t - clkA

        srvbuf = self.sock.recv(128)
        info = bytes.decode(srvbuf[:13], encoding='utf-8')
        if info != 'LNTP-s-finish':
            raise Exception('[Info] unknown error!')
        else:
            self.offset = struct.unpack('d', srvbuf[13:21])[0]

    def lntp_clk(self):
        '''
        :return:  timestamp, datetime object, datetime string
        '''
        timestamp = getclk() + self.er + self.offset
        dt = datetime.datetime.fromtimestamp(timestamp)
        d = str(dt)
        return timestamp,dt,d

def srvtest(addr):
    srv = LNPTServer(addr)
    srv.start()
    
def clnttest(addr):
    clnt = LNTPClient(addr)
    clnt.start()

if __name__ == '__main__':
    import sys
    args = sys.argv
    if args[1] == 'help':
        print('-------------------------------------------')
        print('usage: python lntp.py help, to get help')
        print('usage: python lntp.py server ip port, to start server and bind to ip and port')
        print('usage: python lntp.py client ip port, to start a client connect to the server whith ip and port')
    if len(args)>3:
        if args[1] == 'server':
            ip = args[2]
            port = int(args[3])
            srvtest((ip,port))
        elif args[1] == 'client':
            ip = args[2]
            port = int(args[3])
            clnttest((ip,port))
        else:
            print('-------------------------------------------')
            print('usage: python lntp.py help, to get help')
            print('usage: python lntp.py server ip port, to start server and bind to ip and port')
            print('usage: python lntp.py client ip port, to start a client connect to the server whith ip and port')
    else:
        if args[1] != 'help':
            print('-------------------------------------------')
            print('usage: python lntp.py help, to get help')
            print('usage: python lntp.py server ip port, to start server and bind to ip and port')
            print('usage: python lntp.py client ip port, to start a client connect to the server whith ip and port')















