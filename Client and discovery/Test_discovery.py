from time import sleep
import socket, ipaddress, threading
import psutil

MAX_THREADS = 50
PORT = 4840 

final = {}
def check_port(ip, port):
    """
    Check, if a port of an ip-adress is open or closed
    """
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # TCP
        #sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
        socket.setdefaulttimeout(2.0) # seconds (float)
        result = sock.connect_ex((ip,port))
        if result == 0:
            # print ("PORT is open")
            final[ip] = "OPEN"
        #else:
            # print ("PORT is closed/filtered")
            # final[ip] = "CLOSED"
        sock.close()
    except:
        final[ip] = "EXCEPTION"


if __name__ == "__main__":
    # Get alle network adapters with all network infos
    AllNetworkAdapters = psutil.net_if_addrs()
    
    # Go through all all network adapters and extract only the ipv4 adress and the subnet mask:
    RelevantNetworkInfos = {}
    for adapter in AllNetworkAdapters:
        for configs in AllNetworkAdapters[adapter]:
            # Only the ipv4 config is relevant:
            if configs[0] == socket.AF_INET:
                RelevantNetworkInfos[adapter] = f"{configs.address}/{configs.netmask}"
                
                
    # Scan all connected networks:
    for name in RelevantNetworkInfos:
        
        # Get the network (ipadress/subnetmask)
        network = RelevantNetworkInfos[name]
        
        # Skip the local ip:
        if network == "127.0.0.1/255.0.0.0":
            break
        
        print(f"Scan adapter '{name}'...")
        
        # Multithread through the network and check if the port is available:
        for ip in ipaddress.IPv4Network(network, strict=False): 
            threading.Thread(target=check_port, args=[str(ip), PORT]).start()
            #sleep(0.1)
        
            # limit the number of threads.
            while threading.active_count() > MAX_THREADS :
                sleep(0.1)
        
        
        sorted_ips = dict(sorted(final.items(), key=lambda item: tuple(map(int, item[0].split('.')))))
        
        print(f"Ergebniss für '{name}':")
        for ip, state in sorted_ips.items():
            if state == "OPEN":
                print(f"Found server on {ip}")
                
            
    input("\nPress button to close terminal.....")
