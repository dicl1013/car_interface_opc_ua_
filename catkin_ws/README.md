# Car interface OPC UA
In this project a interface for autonomes vehicles is realised with OPC UA.
For detailes information see the instruction.pdf

# Steps to start the interface
1. If the interfaced is used for the first time on the system or if anything changed, run **init_docker.sh**
2. Start the interface with **start_interface.sh**

# Explanation of important folders/files
## certificates
In this folder are the automaticly created certificate and private key saved.

## config
### mapping_rostopic_opcua.xml
In this file is the mapping from the ROS-topics to the OPC UA nodes parameterized.

Example:
```xml
<ROSUAObject NodeId="ns=2;i=4" RosTopic="Target_1"/>
```
The ROS-Topic **Target_1** is mapped to the OPC UA node with the id **ns=2;i=4**
The datatyp of the ROS-Topic need to match the datatyp of the OPC UA node.

### opcua_model.xml
This file containes the structure of the OPC UA server. It can be modifed by hand or replaced with XML-files which are created with programes like UAModeler or by the script **create_opc_ua_server_struct.py**

### settings.xml
This files contains some general settings for the OPC UA Server.

1. TimeSignInValidInMinutes: Duration how long the username and password is valid since the creation. Needs to be between 1 and 360
2. LengthUsername: Length of the randomly created username. 
3. LengthUsername: Length of the randomly created password.
The combined lenght of username and password needs to be at least 6.

## logfiles
In this folder the logfiles of the interface are saved.

## scripts
In this folder is all python source code

### asyncua
Folder with the modifed python asyncua libary

### create_opc_ua_server_struct.py
This script models an OPC UA server an exports the structure to a XML-file. This XML-file can then be used insted of the opcua_model.xml file in the config folder.

### login_listener.py
Example how to subscribe to the login_listener topic of the ROS-node from the OPC UA Server. On this topic are the current login data published.

### opc_ua_server.py
This file containes all the source code for the OPC UA interface.

# To fix an issue of openssl
    AttributeError: module 'lib' has no attribute 'X509_V_FLAG_CB_ISSUER_CHECK'

```
sudo apt remove python3-pip 
wget https://bootstrap.pypa.io/get-pip.py
sudo python3 get-pip.py
```

And then after a reboot:

```
pip install pyopenssl --upgrade
```