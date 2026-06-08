import socket
from bsonrpc import JSONRpc

# Cut-the-corners TCP Client:
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(('localhost', 6000))

rpc = JSONRpc(s)
server = rpc.get_peer_proxy()
# Execute in server:
result = server.add_goal(0.0,0.0,0.0)
result = server.add_goal(-1.0,0.0,0.0)
while True:

    state = server.get_state()
    print("navigation_state:",state)
rpc.close() # Closes the socket 's' also