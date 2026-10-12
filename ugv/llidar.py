import socket
import json

# UGV Multicast Group and Port 5005 for lidar
MCAST_GRP = "239.255.42.42"
MCAST_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

# Bind to port and join multicast group
sock.bind(("", MCAST_PORT))
mreq = socket.inet_aton(MCAST_GRP) + socket.inet_aton("127.0.0.1")
sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

print(f"Listening for LiDAR datagrams on multicast group {MCAST_GRP}:{MCAST_PORT}...")

try:
    while True:
        data, addr = sock.recvfrom(65535)
        try:
            msg = json.loads(data.decode("utf-8"))
            print(f"[{addr[0]}] Type: {msg.get('type')}, TS: {msg.get('ts')}, Points field type: {type(msg.get('points'))}, Value preview: {str(msg.get('points'))[:80]}")
        except Exception as e:
            print(f"Received raw data (failed JSON parse): {data[:100]} | Error: {e}")
except KeyboardInterrupt:
    print("\nStopped.")