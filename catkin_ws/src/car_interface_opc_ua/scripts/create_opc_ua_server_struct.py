#!/usr/bin/env python

import asyncio

from asyncua import Server
from asyncua.common.xmlexporter import XmlExporter
from asyncua import ua

async def AddObject(Namespace, ParentObject, Objectname, AllNodesList):
    """
    Helper function to add an object to a parent object and append it to the list with all nodes
    """
    NewObject = await ParentObject.add_object(Namespace, Objectname)
    AllNodesList.append(NewObject)

    return NewObject, AllNodesList

async def AddVariable(Namespace, ParentObject, Variablename, InitValue, DataType, AllNodesList):
    """
    Helper function to add a variable to a parent object and append it to the list with all nodes
    """
    NewVariable = await ParentObject.add_variable(Namespace, Variablename, InitValue, DataType)
    AllNodesList.append(NewVariable)

    return NewVariable, AllNodesList

"""-------------------------------------------------------------------------"""
""" Function to create the XML-Model of the OPC UA server"""
""" Alternatively programes like UAModeler can be used """
"""-------------------------------------------------------------------------""" 
async def main():
    # Setup a server:
    server = Server()
    await server.init()

    server.set_endpoint("opc.tcp://0.0.0.0:4840/opcua_server/")

    # Set the namespace:
    #uri = "ThisIsANamespace"
    #idx2 = await server.register_namespace(uri)

    uri = "http://h-ka.de/ieem/car_interface_opc_ua"
    idx = await server.register_namespace(uri)

    # Create the server struct:
    AllNodes = []
    # Create main car object:
    ObjCar, AllNodes = await AddObject(idx, server.nodes.objects, "Car", AllNodes)
    
    # Create object for all the sensors:
    ObjSensors, AllNodes = await AddObject(idx, ObjCar, "Sensors", AllNodes)

    # Lidar:
    ObjSensor1, AllNodes = await AddObject(idx, ObjSensors, "Lidar", AllNodes)
    #_, AllNodes = await AddVariable(idx, ObjSensor1, "AllTargets", [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]], None, AllNodes)
    _, AllNodes = await AddVariable(idx, ObjSensor1, "Target_1", "Init", None, AllNodes)
    _, AllNodes = await AddVariable(idx, ObjSensor1, "Target_2", "Init", None, AllNodes)

    # Camera01:
    ObjSensor2, AllNodes = await AddObject(idx, ObjSensors, "Camera01", AllNodes)
    _, AllNodes = await AddVariable(idx, ObjSensor2, "DetectedCirclesFlag", 0, None, AllNodes)
    _, AllNodes = await AddVariable(idx, ObjSensor2, "Distance", 0, None, AllNodes)
    #_, AllNodes = await AddVariable(idx, ObjSensor2, "Image", 1, ua.VariantType.ByteString, AllNodes)

    # Camera02:
    ObjSensor3, AllNodes = await AddObject(idx, ObjSensors, "Camera02", AllNodes)
    _, AllNodes = await AddVariable(idx, ObjSensor3, "DetectedCirclesFlag", 0, None, AllNodes)
    _, AllNodes = await AddVariable(idx, ObjSensor3, "Distance", 0, None, AllNodes)

    # Export the server in a xml file:
    exporter = XmlExporter(server)
    await exporter.build_etree(AllNodes)
    await exporter.write_xml('ua-export.xml')

    """
    async with server:
        while True:
            await asyncio.sleep(1)
    """

if __name__ == "__main__":
    asyncio.run(main())
