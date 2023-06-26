import socket
import json
import time

#v1.2


def connectETController(ip,port=8055):
  sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  sock.settimeout(2)
  try:
    sock.connect((ip,port))
    return (True,sock)
  except Exception as e:
    sock.close()
    return (False, None)


def disconnectETController(sock):
  if(sock):
    sock.close()
    sock=None
  else:
    sock=None


def sendCMD(sock,cmd,params=None,id=1) -> object:
  if(not params):
    params=[]
  else:
    params=json.dumps(params)
  sendStr="{{\"method\":\"{0}\",\"params\":{1},\"jsonrpc\":\"2.0\",\"id\":{2}}}".format(cmd,params,id)+"\n"
  try:
    #print(sendStr)
    sock.sendall(bytes(sendStr,"utf-8"))
    ret=sock.recv(1024)
    jdata=json.loads(str(ret,"utf-8"))
    if("result" in jdata.keys()):
      return (True,json.loads(jdata["result"]),jdata["id"])
    elif("error" in jdata.keys()):
      print(jdata['error']['message'])
      return (False,json.loads(jdata["error"]),jdata["id"])
    else:
      return (False,None,None)
  except Exception as e:
    return (False,None,None)


if __name__ == "main":  # 机 器 人 IP 地 址
    pass