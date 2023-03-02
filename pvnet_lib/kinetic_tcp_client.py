import socket
import json
import queue
import threading
import time

myhost = socket.gethostbyname(socket.gethostname())
print(myhost)


class kinetic_tcp_client:
    def __init__(self, host=myhost, port=8080):
        self.host = host  # 主机IP
        self.port = port
        # 端口

        self.inputData = queue.Queue(1024)
        self.t = threading.Thread(target=self.rec_data)
        self.cnt_state = False

    def start_connection(self):
        while True:
            try:
                self.web = socket.socket()  # 创建TCP/IP套接字
                self.web.connect((self.host, self.port))
            except Exception as message:
                print('连接服务器报错%s' % message)
                time.sleep(1)
                continue
            else:
                self.cnt_state = True
                break

    def close_connection(self):
        self.web.close()

    def try_connect_srv(self):
        while True:
            try:
                self.start_connection()
                data = {'hello': True,
                        'command': None}
                jdata = json.dumps(data)
                self.send_data(jdata)
                time.sleep(1)
                try:
                    data = self.web.recv(1024 * 10).decode()  # 获取客户端请求的数据
                except Exception as message:
                    print('消息接受失败%s' % message)
                else:
                    jdata = json.loads(data)

                    print('连接成功')
                    print('收到数据: ', jdata)
                    break

            except Exception as message:
                print('消息发送失败%s' % message)
                time.sleep(2)
                if self.cnt_state:
                    self.web.close()
                    self.cnt_state = False

    def rec_data(self):
        while True:
            data = self.web.recv(1024 * 10).decode()  # 获取客户端请求的数据
            if data:
                data_json = json.loads(data)
                self.inputData.put(data_json)

    def send_data(self, json_data):
        byte_json = json_data.encode()
        self.web.send(byte_json)

    def start_rec(self):
        self.t.start()


if __name__ == '__main__':
    my_cnt = kinetic_tcp_client()
