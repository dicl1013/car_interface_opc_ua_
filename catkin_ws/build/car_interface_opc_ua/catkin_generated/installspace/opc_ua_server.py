#!/usr/bin/env python3

"""
Author: Clemens Diener
E-Mail: dicl1013@h-ka.de / cl.diener@t-online.de
Date: winter semester 2023/24

This script creates an OPC UA server and connects ROS-Topics with OPC UA nodes. So it is possible to read data from the ROS-network from outside.
The OPC UA Server modul is in config/opcua_model.xml and the mapping of the ROS-topics to the nodes is done in config/mapping_rostopic_opcua.xml
"""

"""-------------------------------------------------------------------------"""
""" Imports:"""
"""-------------------------------------------------------------------------""" 
# ROS-packages:
import rospy
import std_msgs.msg
import sensor_msgs.msg

# Packages for OPC UA:
import asyncio
import logging
from typing import Union

# Add file paths of custom modules:
import os
print(os.getcwd())

import sys
sys.path.append("src/car_interface_opc_ua/scripts")
print(sys.path)

#from asyncua import Server, ua
from OwnAsyncua import Server, ua
#import OwnAsyncua.server as Server
#import OwnAsyncua.ua as ua
#from asyncua.server.users import User, UserRole
from OwnAsyncua.server.users import User, UserRole
#import OwnAsyncua.server.users.User as User
#import OwnAsyncua.server.users.UserRole as UserRole
#from asyncua.crypto.cert_gen import setup_self_signed_certificate
from OwnAsyncua.crypto.cert_gen import setup_self_signed_certificate
from cryptography.x509.oid import ExtendedKeyUsageOID
from pathlib import Path

import socket
import datetime

# Queue for thread safe queue:
# (Queue's from asyncio are not thread safe)
import queue

# Package for XML-parsing:
import xml.etree.ElementTree as ET

# Warnings:
import warnings

# For User managment:
import random
import string

# Dataclass for Struct:
import dataclasses

"""-------------------------------------------------------------------------"""
""" Constants:"""
"""-------------------------------------------------------------------------""" 
# TODO: Make Filepaths constants dynamic
# Filepaths of xml fils:
FILEPATH_SERVERMODEL = "src/car_interface_opc_ua/config/opcua_model.xml"
FILEPATH_MAPPING_ROSTOPIC_OPCUA = "src/car_interface_opc_ua/config/mapping_rostopic_opcua.xml"
FILEPATH_SETTINGS = "src/car_interface_opc_ua/config/settings.xml"

# Certificate and private key path:
FILEPATH_SERVER_CERTIFICATE = "src/car_interface_opc_ua/certificates/server-certificate.der"
FILEPATH_SERVER_PRIVATE_KEY = "src/car_interface_opc_ua/certificates/server-private-key.pem"

"""-------------------------------------------------------------------------"""
""" Settings-Struct:"""
"""-------------------------------------------------------------------------""" 
@dataclasses.dataclass
class SettingsDataclass:
    # Sign in data:
    MaxTimeLogInData: float = 100  # In minutes
    LengthUsername: int = 4
    LengthUsername: int = 4

"""-------------------------------------------------------------------------"""
""" Own UserManager"""
""" Needed, because Python package don't have login with Username and password"""
"""-------------------------------------------------------------------------""" 
class UserManager:
    """
    Standard User functionality
    """
    def __init__(self):
        """
        Init UserManager
        """
        self._all_user = {}
    
    async def add_role(self, user_role: UserRole, name: str, pw: str, format: Union[str, None] = None):
        """
        Add an user with the given role
        """
        # Check the name:
        if name is None:
            raise KeyError
    
        # Create an user instance:
        user = User(role=user_role, name=name)
    
        # If the username already exist:
        if name in self._all_user:
            logging.warning(f"User with name {name} "
                            f"attempted to be added multiple times, only the last user will be kept.")
    
        # Add user with the password to the list:
        self._all_user[name]={'pw': pw, 'name': name, 'user': user}
        
    def get_user(self, iserver, username=None, password=None, certificate=None):
        """
        Get the user with the given username und password. If no correct user is given, return None
        """
        if username is None:
            return None
        
        # Find the user with the correct name and password
        correct_users = [user['user'] for user in self._all_user.values()
                         if ((username == user['name']) and (password == user['pw']))]
                
        # If no user was found:
        if len(correct_users) == 0:
            return None
        # If an user was found:
        else:
            return correct_users[0]
    
    def exist_user(self, user=None):
        """
        Checks if an user with the given username exist
        Used for deny acess, if an user was deleted
        """
        if user is None:
            return False
        
        # Extract the username
        username = user.name
        
        # Find the user with the correct name:
        correct_users = [user['user'] for user in self._all_user.values()
                         if (username == user['name'])]
                
        # If no user was found:
        if len(correct_users) == 0:
            return False
        # If an user was found:
        else:
            return True

    async def add_user(self, name: str, pw: str, format: Union[str, None] = None):
        """
        Add a normal user
        """
        await self.add_role(user_role=UserRole.User, name=name, pw=pw, format=format)

    async def add_admin(self, name: str, pw: str, format: Union[str, None] = None):
        """
        Add a admin user
        """
        await self.add_role(user_role=UserRole.Admin, name=name, pw=pw, format=format)

    async def delete_user(self, name: str):
        """
        Delete the user with the given name
        """
        del self._all_user[name]

"""-------------------------------------------------------------------------"""
""" Helper Functions:"""
"""-------------------------------------------------------------------------""" 
def CheckDataTyp(DataType):
    """
    Checks if the given Datatype as String is a valid DataType
    return True if it is valid
    """
    AllValidDataTypes = ["Boolean", 
                         "SByte",
                         "Byte",
                         "Int16",
                         "UInt16",
                         "Int32",
                         "UInt32",
                         "Int64",
                         "UInt64",
                         "Float",
                         "Double",
                         "String",
                         "ByteString"]
    # If the given DataType is in the list, return True:
    for ValidDataType in AllValidDataTypes:
        if DataType == ValidDataType:
            return True
    
    # Otherwise return False:
    return False

def DataTypeString_to_DataTypeRospy(DataTypeString: str):
    """
    Return the std_msgs class of the given DataType
    """
    if DataTypeString == "Boolean":
        return std_msgs.msg.Bool
    elif DataTypeString == "SByte":
        return std_msgs.msg.UInt8
    elif DataTypeString == "Byte":
        return std_msgs.msg.Int8
    elif DataTypeString == "Int16":
        return std_msgs.msg.Int16
    elif DataTypeString == "UInt16":
        return std_msgs.msg.UInt16
    elif DataTypeString == "Int32":
        return std_msgs.msg.Int32
    elif DataTypeString == "UInt32":
        return std_msgs.msg.UInt32
    elif DataTypeString == "Int64":
        return std_msgs.msg.Int64
    elif DataTypeString == "UInt64":
        return std_msgs.msg.UInt64
    elif DataTypeString == "Float":
        return std_msgs.msg.Float32
    elif DataTypeString == "Double":
        return std_msgs.msg.Float64
    elif DataTypeString == "String":
        return std_msgs.msg.String    
    elif DataTypeString == "SByteMultiArray":
        return std_msgs.msg.UInt8MultiArray
    elif DataTypeString == "ByteMultiArray":
        return std_msgs.msg.Int8MultiArray
    elif DataTypeString == "Int16MultiArray":
        return std_msgs.msg.Int16MultiArray
    elif DataTypeString == "UInt16MultiArray":
        return std_msgs.msg.UInt16MultiArray
    elif DataTypeString == "Int32MultiArray":
        return std_msgs.msg.Int32MultiArray
    elif DataTypeString == "UInt32MultiArray":
        return std_msgs.msg.UInt32MultiArray
    elif DataTypeString == "Int64MultiArray":
        return std_msgs.msg.Int64MultiArray
    elif DataTypeString == "UInt64MultiArray":
        return std_msgs.msg.UInt64MultiArray
    elif DataTypeString == "FloatMultiArray":
        return std_msgs.msg.Float32MultiArray
    elif DataTypeString == "DoubleMultiArray":
        return std_msgs.msg.Float64MultiArray
    elif DataTypeString == "ByteString":
        return sensor_msgs.msg.Image
    else:
        raise RuntimeError("Unknown Datatype")

async def ParseRosTopicsUANodes(FilePathMapping, FilePathModel):
    """
    Function to extract the relevant ROS-topic of the xml file.
    Returns a list with all mapped topics
    """
    # Read the mapping XML file:
    with open(FilePathMapping, "r") as file:
        XMLDataMapping = file.read()

    # Analyse the data:
    rootMapping = ET.fromstring(XMLDataMapping)

    # Create empty list for the Rostopics and NodeIds:
    RosTopics_UANodes = []
    RelevantOPCUANodes = []

    # Go through the mapping-xml:
    for ROSUAObject in rootMapping.iter("ROSUAObject"):
        # Get the elements:
        RosTopic = ROSUAObject.get("RosTopic")
        NodeId = ROSUAObject.get("NodeId")

        # Check if XML is correct:
        if RosTopic == None:
            raise RuntimeError(f"RosTopic is missing in the XML file! {FilePathMapping}")
        if NodeId == None:
            raise RuntimeError(f"NodeId is missing in the XML file! {FilePathMapping}")

        # Append the mapped data to the list:
        RosTopics_UANodes.append([RosTopic, NodeId])

        # Append the relevant NodeID for the data type extraction:
        RelevantOPCUANodes.append(NodeId)
    
    # Raise warning, if no mapped data exist:
    # Remaining source code is irrelevant
    if len(RosTopics_UANodes) == 0:
        warnings.warn("No mapped RosTopics found")
        return RosTopics_UANodes

    # Read the model XML file:
    with open(FilePathModel, "r") as file:
        XMLDataModel = file.read()

    # Analyse the data:
    rootModel = ET.fromstring(XMLDataModel)

    # Extract the relevant namespace:
    namespace = rootModel.tag.split("}")[0][1:]
    namespace = {"ns": namespace}

    # Go through all relevant NodeIDs:
    for i, NodeId in enumerate(RelevantOPCUANodes):
        # Helper variable to indicate, if the node was found:
        FoundIt = False

        # Find the Correct Node:
        for UAVariable in rootModel.findall("ns:UAVariable", namespace):

            # If the correct UAVariable is found:
            if UAVariable.get("NodeId") == NodeId:
                DataType = UAVariable.get("DataType")

                # Check if the DataType of the node is valid:
                if not CheckDataTyp(DataType):
                    raise RuntimeError(f"Invalid or not supported Datatype at Node {NodeId} in {FilePathModel}")   

                # Check if the datatyp is an array:
                ValueRank = UAVariable.get("ValueRank")
                if ValueRank != None:
                    DataType = DataType + "MultiArray"  # type: ignore

                # If valid, append DataType and continue with next NodeID:
                RosTopics_UANodes[i].append(DataType)
                FoundIt = True
                break 
        
        # If the NodeID was not found in the XML file, raise error:
        if not FoundIt:
            raise RuntimeError(f"The Node {NodeId} is missing in {FilePathModel}")
        
    return RosTopics_UANodes

async def ParseSettings(FileSettings):
    """
    Function to parse the settings.xml
    """
    # Create a instance of the settings-struct:
    Settings = SettingsDataclass()

    # Read the XML file:
    with open(FileSettings, "r") as file:
        XMLData = file.read()

    # Analyse the data:
    root = ET.fromstring(XMLData)

    XMLSignInData = root.find('SignIn')

    # Get the time how long the sign in data are valid:
    TimeSignInValidInMinutes = float(XMLSignInData.find("TimeSignInValidInMinutes").text) # type: ignore

    # Check if duation is valid:
    if TimeSignInValidInMinutes < 1:
        logging.warn("TimeSignInValidInMinutes is shorter than 1 minute! Value is set to 1")
        Settings.MaxTimeLogInData = 1
    elif TimeSignInValidInMinutes > 360:
        logging.warn("TimeSignInValidInMinutes is longer than 360 minutes! Value is set to 360")
        Settings.MaxTimeLogInData = 360
    else:
        logging.info(f"TimeSignInValidInMinutes is set to {TimeSignInValidInMinutes}")
        Settings.MaxTimeLogInData = TimeSignInValidInMinutes

    # Extract the length of the Sign in data:
    LengthUsername = int(XMLSignInData.find("LengthUsername").text) # type: ignore
    LengthUsername = int(XMLSignInData.find("LengthUsername").text) # type: ignore

    # Make sure, not to unsafe Sign in data are set:
    if (LengthUsername + LengthUsername) < 6:
        logging.warn("Overall length of username and password to short! LengthUsername is set to 3. LengthUsername is set to 3.")
        Settings.LengthUsername = 3
        Settings.LengthUsername = 3
    elif LengthUsername < 1:
        logging.warn("LengthUsername is shorter than 1. LengthUsername is set to 1.")
        Settings.LengthUsername = 1
        Settings.LengthUsername = LengthUsername
    elif LengthUsername < 1:
        logging.warn("LengthUsername is shorter than 1. LengthUsername is set to 1.")
        Settings.LengthUsername = LengthUsername
        Settings.LengthUsername = 1
    else:
        logging.info(f"LengthUsername is set to {LengthUsername}, LengthUsername is set to {LengthUsername},")
        Settings.LengthUsername = LengthUsername
        Settings.LengthUsername = LengthUsername

    return Settings

"""-------------------------------------------------------------------------"""
""" class OPCUAServer_RosNode():"""
"""-------------------------------------------------------------------------""" 
class OPCUAServer_RosNode():
    """
    Class for the ROS-Node of the OPC UA server
    """
    def __init__(self, DataQueIn, DataQueOut, currentDateString):
        # Queue for Communication:
        self.DataQueIn = DataQueIn
        self.DataQueOut = DataQueOut

        # Current date as string for logfile name
        self.currentDateString = currentDateString

        # Create Lst with the data of the login:
        self.CurrentLogInData = {"Username" : 'NoCurrentUsername',
                                 "Password" : 'NoCurrentPassword',
                                 "Client" : 'NorCurrentClient',
                                 "Timestamp" : datetime.datetime.now(),
                                 "Valid" : False}
    
        # Init a Settings instance
        self.Settings = SettingsDataclass()

    def callback(self, data, UANode):
        """
        Callback Function of the ros node.
        Parameters: 
            - data: data from the subscribed topic
            - UANode: NodeID of the OPC-UA server, where the data should be written
        """
        # If the recieved data is an array:
        if data._slot_types[0] == 'std_msgs/MultiArrayLayout':
            # Convert tuple to list:
            data.data = list(data.data)

        # Put the new data with the UA node where it should be published on the queue:
        self.DataQueOut.put_nowait(["PublishData", UANode, data.data])

    def logMsg(self, msg):
        """
        Function to log data an print it to the console
        """
        rospy.loginfo(msg)
        self.logger.info(msg)

    async def SetupNode(self, RosTopics_UANodes, Settings):
        """
        Setup the ROS-Node
        """
        # Setup the logger:
        self.logger = logging.getLogger("RosNode")
        self.logger.setLevel(logging.DEBUG)
        logger_handler = logging.FileHandler(f"src/car_interface_opc_ua/logfiles/logfile_RosNode_{self.currentDateString }.log")
        logger_formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        logger_handler.setFormatter(logger_formatter)
        self.logger.addHandler(logger_handler)

        # Set the settings:
        self.Settings = Settings

        # Init the node:
        self.logMsg("Init node 'OPCUA_RosNode'...")
        self.RosPublisher = rospy.Publisher('OPCUA_LoginData', std_msgs.msg.String, queue_size=10)
        rospy.init_node('OPCUA_RosNode', anonymous=False)

        # Subscribe to all relevant topics:
        for topic, UANode, DataType in RosTopics_UANodes:
            self.logMsg(f"Subscribe {topic} to {UANode}")
            #rospy.get_param # TODO: WOHER??
            rospy.Subscriber(topic, DataTypeString_to_DataTypeRospy(DataType), self.callback, UANode)

    async def CreateRandomString(self, length):
        """
        Return a random string with given length
        """
        PossibleSymbols = string.ascii_letters + string.digits
        RandomString = ''.join(random.choice(PossibleSymbols) for _ in range(length))
        return RandomString

    async def CheckUserValidationTime(self):
        """
        Check if the current user is time valid
        """
        # Get current time and calculate delta time:
        CurrentTime = datetime.datetime.now()
        deltaTimeMinutes = (CurrentTime - self.CurrentLogInData["Timestamp"]).total_seconds() / 60

        # If first connection to far in the past, delete the user:
        if (deltaTimeMinutes > self.Settings.MaxTimeLogInData) and self.CurrentLogInData["Valid"] :
            self.CurrentLogInData["Valid"] = False
            self.DataQueOut.put(["DeleteUser"])


    async def HandleNewConnection(self, ClientString):
        """
        Function to handle the event of a new connection
        """
        # Calculate time since last new connection:
        CurrentTime = datetime.datetime.now()
        deltaTimeMinutes = (CurrentTime - self.CurrentLogInData["Timestamp"]).total_seconds() / 60
        
        # If other Client connected than last time or to long since the last connect,
        # then create a new account with new password:
        if (ClientString != self.CurrentLogInData["Client"]) or deltaTimeMinutes > self.Settings.MaxTimeLogInData:
            Username = await self.CreateRandomString(self.Settings.LengthUsername)
            Password = await self.CreateRandomString(self.Settings.LengthUsername)
            self.CurrentLogInData["Username"] = Username
            self.CurrentLogInData["Password"] = Password
            self.CurrentLogInData["Client"] = ClientString
            self.CurrentLogInData["Timestamp"] = CurrentTime
            self.CurrentLogInData["Valid"] = True

        # Send login data to the OPC UA server:
        self.DataQueOut.put(["LoginData", self.CurrentLogInData["Username"], self.CurrentLogInData["Password"]])
        # Publish the Logi Data in the ros network:
        self.RosPublisher.publish(f"{ClientString};{self.CurrentLogInData['Username']};{self.CurrentLogInData['Password']};{self.CurrentLogInData['Timestamp']};{self.Settings.MaxTimeLogInData}")


    async def run(self):
        while not rospy.is_shutdown():
            # Try to extract data from the queue:
            data = None
            try:
                data = self.DataQueIn.get_nowait()  
            except:
                # If no data on the queue, sleep for 10 ms and handle other data
                await asyncio.sleep(0.01)  

            if data:
                self.logMsg(data)

                if data[0] == "NewConnection":
                    print(f"New Connection from: {data[1]}")
                    await self.HandleNewConnection(data[1])

            await self.CheckUserValidationTime()

"""-------------------------------------------------------------------------"""
""" class OPCUAServer():"""
"""-------------------------------------------------------------------------""" 
class OPCUAServer():
    """
    Class for the OPC-UA Server of the ROS-Node
    """
    def __init__(self, DataQueIn, DataQueOut, currentDateString):
        # Queue for Communication:
        self.DataQueIn = DataQueIn
        self.DataQueOut = DataQueOut

        # Current date as string for logfile name
        self.currentDateString = currentDateString

        # Maybe late in file:
        self.product_uri = "urn:hka:ubuntu:Car_interface_OPC_UA"
        self.manufacturer_name = "Hochschule Karlsruhe - University of Applied Sciences (HKA)"
        self.product_name = "Car interface - OPC UA"
        self.software_version = "1.0"
        self.build_number = "0"
        self.build_date = datetime.datetime.now()    
          
        # Create logfile an log to this file:
        logging.basicConfig(filename=f"src/car_interface_opc_ua/logfiles/logfile_OPCUA_{self.currentDateString}.log", level=logging.DEBUG, 
                            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s', force=True)
        

    def logMsg(self, msg):
        """
        Function to log data and print it to the console
        """
        rospy.loginfo(msg)
        #logging.info(msg)

    def SetupServer(self):
        """
        Setup the Server
        """

    async def RunOPCUA(self):
        """
        Main Function of the OPC UA Server
        """
        self.logMsg("Init OPC UA server...")
        # Setup the users:
        # Create list with the data of the login
        # Will be overwritten with random login data:
        self.CurrentLogInData = {"Username" : 'Tester',
                                 "Password" : 'TesterPW'}
        user_manager = UserManager()
        await user_manager.add_user(self.CurrentLogInData["Username"], self.CurrentLogInData["Password"])

        # Setup the server with the user manager:
        server = Server(user_manager=user_manager)
        await server.init()

        # Set the build info.
        await server.set_build_info(self.product_uri, 
                                    self.manufacturer_name, 
                                    self.product_name, 
                                    self.software_version, 
                                    self.build_number, 
                                    self.build_date)

        # Import the server structure:
        await server.import_xml(FILEPATH_SERVERMODEL)

        # Set the ip-address so the server is available over the network adapter address:
        server.set_endpoint("opc.tcp://0.0.0.0:4840/opcua_server/")

        # Set the name of the server:
        server.set_server_name("Car interface OPC UA Server")

        # Set the application URI:
        host_name = socket.gethostname()    # Hostname needed, so no 'BadCertificateUriInvalid' error occure on the client
        ServerURI = f"urn:hka:{host_name}:Car_interface_OPC_UA/server"
        await server.set_application_uri(ServerURI)

        # Set all possible endpoint policies for clients to connect through:
        server.set_security_policy(
            [
                #ua.SecurityPolicyType.NoSecurity,
                ua.SecurityPolicyType.Basic256Sha256_SignAndEncrypt,
                #ua.SecurityPolicyType.Basic256Sha256_Sign,
            ]
        )

        # Below is only required if the server should generate its own certificate,
        # It will renew also when the valid datetime range is out of range (on startup, no on runtime)
        await setup_self_signed_certificate(Path(FILEPATH_SERVER_PRIVATE_KEY),
                                            Path(FILEPATH_SERVER_CERTIFICATE),
                                            ServerURI,
                                            socket.gethostname(),
                                            [ExtendedKeyUsageOID.SERVER_AUTH],
                                            {
                                                'countryName': 'DE',
                                                'stateOrProvinceName': 'Baden-Württemberg',
                                                'localityName': 'Karlsruhe',
                                                'organizationName': "Hochschule Karlsruhe - University of Applied Sciences (HKA)"
                                            })

        # Load server certificate and private key. This enables endpoints with signing and encryption.
        await server.load_certificate(FILEPATH_SERVER_CERTIFICATE)
        await server.load_private_key(FILEPATH_SERVER_PRIVATE_KEY)

        self.logMsg("Starting OPC UA server...")

        # Init list for client connection checking:
        self.clientsAlt = []

        async with server:
            # Run Server as long ROS-Node is active: 
            while not rospy.is_shutdown():
                # Try to extract data from the queue:
                data = None
                try:
                    data = self.DataQueIn.get_nowait()  
                except:
                    # If no data on the queue, sleep for some time and handle the server:
                    await asyncio.sleep(0.005)  

                # If data is available:
                if data:
                    self.logMsg(f"OPCUAServer: {data}")

                    if data[0] == "PublishData":
                        # Extract data:
                        UANode = data[1]
                        value = data[2]
                        # Write data to the given NodeID of the server:
                        self.logMsg(f"Set value of {UANode} to {value}")
                        await server.get_node(UANode).write_value(value)     

                    elif data[0] == "DeleteUser":
                        self.logMsg(f"DeleteUser")
                        # Delete the current user:
                        await server.iserver.user_manager.delete_user(self.CurrentLogInData["Username"]) # type: ignore

                    elif data[0] == "LoginData":
                        Username = data[1]
                        Password = data[2]
                        #self.logMsg(f"Username: {Username}, Password: {Password}")

                        # If Current Login data does not match the given login data:
                        if Username != self.CurrentLogInData["Username"] or Password != self.CurrentLogInData["Password"]:
                            # Delete the current user and create a new one with the given data:
                            # Additionally to "DeleteUser" needed, so if another client connects, only one user exist
                            await server.iserver.user_manager.delete_user(self.CurrentLogInData["Username"]) # type: ignore

                            self.CurrentLogInData["Username"] = Username
                            self.CurrentLogInData["Password"] = Password
                            await server.iserver.user_manager.add_user(self.CurrentLogInData["Username"], self.CurrentLogInData["Password"]) # type: ignore

                # Check if a new connection were made:
                if self.clientsAlt != server.bserver.clients: # type: ignore
                    # If new connection is made:
                    #if len(self.clientsAlt) == 0 and len(server.bserver.clients) == 1: # type: ignore
                    if len(self.clientsAlt) < len(server.bserver.clients):  # type: ignore
                         # Get Information about the new client:
                        ClientIP = server.bserver.clients.__str__()         # String looks like this: [OPCUAProtocol(('192.168.233.1', 58133), None)] # type: ignore
                        
                        # Extract the IP-adress for client identification 
                        ClientIP = ClientIP[(ClientIP.find("'") + 1):]      # Cut the text before the ip adress
                        ClientIP = ClientIP[:ClientIP.find("'")]            # Cut the text after the ip adress

                        # Try to put the new Connection on the queue to the ROS node:
                        try:
                            self.DataQueOut.put(["NewConnection", ClientIP], timeout=0.1)
                        except:
                            self.logMsg("You should never see this message! Queue from OPC UA Server to the ROS node is full!")
                            
                    self.clientsAlt = server.bserver.clients.copy() # type: ignore

            self.logMsg("OPC UA Ende")


"""-------------------------------------------------------------------------"""
""" async def main():"""
"""-------------------------------------------------------------------------""" 
async def main():
    """
    Main function
    """
    # Create String with the current datetime for the name of the logfile:
    currentDate = datetime.datetime.now()
    currentDateString = currentDate.strftime("%Y-%-m-%d %H:%M:%S")    

    # Create Queue for communications between threads:
    DataQueRosToOPC = queue.Queue()
    DataQueOPCToRos = queue.Queue(maxsize=10) # Limit the size to make brute force complete impossible

    # Create ROS-Node and the OPC-UA server:
    RosNode = OPCUAServer_RosNode(DataQueOPCToRos, DataQueRosToOPC, currentDateString)
    OPCUAServer1 = OPCUAServer(DataQueRosToOPC, DataQueOPCToRos, currentDateString)

    # Extract the settings from the xml:
    Settings = await ParseSettings(FILEPATH_SETTINGS)

    # Extract list of relevant topics:
    RosTopics_UANodes = await ParseRosTopicsUANodes(FILEPATH_MAPPING_ROSTOPIC_OPCUA, FILEPATH_SERVERMODEL)

    # Initiate the ROS-Node:
    await RosNode.SetupNode(RosTopics_UANodes, Settings)

    # Create the threads:    
    task1 = asyncio.create_task(RosNode.run())
    task2 = asyncio.create_task(OPCUAServer1.RunOPCUA())

    # Run the threads:
    await asyncio.gather(task2, task1)
    rospy.loginfo("Gather Ende")

if __name__ == "__main__":
    # Get current date:
    asyncio.run(main(), debug=True)
