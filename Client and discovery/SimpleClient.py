
"""
Author: Clemens Diener

A simple OPC UA client, to test the funcionality of the OPC UA server

"""

import asyncio
import logging
import sys
import socket
from pathlib import Path
from cryptography.x509.oid import ExtendedKeyUsageOID
sys.path.insert(0, "..")
from asyncua import Client
from asyncua.crypto.security_policies import SecurityPolicyBasic256Sha256
from asyncua.crypto.cert_gen import setup_self_signed_certificate
from asyncua.crypto.validator import CertificateValidator, CertificateValidatorOptions
from asyncua.crypto.truststore import TrustStore
from asyncua import ua

import numpy as np

import os
os.system('color')

import time

# Constants for deviation:
TOLERANCE_X = 0.05
TOLERANCE_Y = 0.05
TOLERANCE_Z = 0.05

# Constantes for security:
USE_TRUST_STORE = True
cert_base = Path(__file__).parent
FILEPATH_CLIENT_CERTIFICATE = Path(cert_base / "certificates/client-certificate.der")
FILEPATH_CLIENT_PRIVATE_KEY = Path(cert_base / "certificates/client-private-key.pem")


class bcolors:
    """
    Class for color codes, for better print statements
    """
    ENDC = '\033[0m'
    PASS = '\033[32m'   # Red
    FAIL = '\033[31m'   # Green
    WARNING = '\033[93m'
    

async def TaskRunClient(loop):
    """
    Function to run the client
    """
    # setup logging:    
    logging.basicConfig(level=logging.INFO)

    # Get the hostname for the certificate:    
    host_name = socket.gethostname()
    client_app_uri = f"urn:{host_name}:SimpleTestClient"

    # Setup the client cerificate:
    # If there is no certificate, then create one:
    await setup_self_signed_certificate(FILEPATH_CLIENT_PRIVATE_KEY,
                                        FILEPATH_CLIENT_CERTIFICATE,
                                        client_app_uri,
                                        host_name,
                                        [ExtendedKeyUsageOID.CLIENT_AUTH],
                                        {
                                            'countryName': 'DE',
                                            'stateOrProvinceName': 'Baden-Württemberg',
                                            'localityName': 'Karlsruhe',
                                            'organizationName': "Hochschule Karlsruhe - University of Applied Sciences (HKA)",
                                        })
    
    
    # Get the ip-adress:
    IpAdressServer = input("Ip-Adress of the wanted OPC UA server: ")
    server_url = f"opc.tcp://{IpAdressServer}:4840/freeopcua/server/"

    # Create the client instance:
    client = Client(url=server_url)
    client.application_uri = client_app_uri

    # Set the security policy:
    await client.set_security(
        SecurityPolicyBasic256Sha256,
        certificate=str(FILEPATH_CLIENT_CERTIFICATE),
        private_key=str(FILEPATH_CLIENT_PRIVATE_KEY)
    )

    # Get the validator for the certificate:
    if USE_TRUST_STORE:
        trust_store = TrustStore([Path('examples') / 'certificates' / 'trusted' / 'certs'], [])
        await trust_store.load()
        validator = CertificateValidator(CertificateValidatorOptions.TRUSTED_VALIDATION|CertificateValidatorOptions.PEER_SERVER, trust_store)
    else:
        validator = CertificateValidator(CertificateValidatorOptions.EXT_VALIDATION|CertificateValidatorOptions.PEER_SERVER)
    client.certificate_validator = validator
    
    # Input for username and password:
    username = input("Username:")
    password = input("Password:")
    client.set_user(username)
    client.set_password(password)  
 
    try:
        async with client:
            # Get the both Target nodes of the lidar:
            Target1Node = client.get_node('ns=2;i=4')
            Target2Node = client.get_node('ns=2;i=5')
            
            # Forever:
            while True:
                # Try to read the file with the target positions:
                try:
                    with open("SetPosition.csv", "r") as file:
                        TargetData = file.read()
                        
                    Target1SetPos = TargetData.split(";")
                    Target1SetPos = [float(Target1SetPos[0]), float(Target1SetPos[1]), float(Target1SetPos[2])]
                                    
                    # Print the set positiion:
                    print(f"Set position: {Target1SetPos[0]} / {Target1SetPos[1]} / {Target1SetPos[2]}")
                
                except:                    
                    # Input the set coordinates:
                    try:
                        Target1SetX = float(input("Target1 set position X: "))
                        Target1SetY = float(input("Target1 set position Y: "))
                        Target1SetZ = float(input("Target1 set position Z: "))
                        # Put them together:
                        Target1SetPos = [Target1SetX, Target1SetY, Target1SetZ]
                    except:
                        print("Invalid Input!")
                        Target1SetPos = [0.0, 0.0, 0.0]
                Target1ActualValueString = await Target1Node.read_value()
    
                if Target1ActualValueString == "NOT FOUND":
                    print(f"{bcolors.WARNING}Target not found!{bcolors.ENDC}\n\n")
                else:                    
                    # Get the actual position for target 1:
                    # Actual Position is a String, with the coordinates seperated by '/'
                    Target1ActualPos = Target1ActualValueString.split("/")
                    # Convert the string to float:
                    Target1ActualPos = [float(Target1ActualPos[0]), float(Target1ActualPos[1]), float(Target1ActualPos[2])]
                    
                    # Compare actual and set position:
                    await ComparePos(Target1ActualPos, Target1SetPos)
                    
                time.sleep(1)
                
    except ua.UaError as exp:
        logging.error(exp)

async def ComparePos(ActualPos, SetPos):
    """
    Compare the Actual Pos and the Set pos
    """
    # Calculate the delta position:
    # Use numpy array for that, so '-' can used on the whole array:
    deltaPos = np.array(SetPos) - np.array(ActualPos)

    # Print the actual position:
    print(f"Actual position: {ActualPos[0]} / {ActualPos[1]} / {ActualPos[2]}")
    
    # Check if the Axis are in tolerance:
    AllPosInTolerance = True
    if abs(deltaPos[0]) > TOLERANCE_X:
        AllPosInTolerance = False
        print(f"The X position deviates too much! ({deltaPos[0]})")

    if abs(deltaPos[1]) > TOLERANCE_Y:
        AllPosInTolerance = False
        print(f"The Y position deviates too much! ({deltaPos[1]})")
        
    if abs(deltaPos[2]) > TOLERANCE_Z:
        AllPosInTolerance = False
        print(f"The Z position deviates too much! ({deltaPos[2]})")
        
    # Print the result:
    if AllPosInTolerance:
        Result = f"{bcolors.PASS}PASS{bcolors.ENDC}"
    else:
        Result = f"{bcolors.FAIL}FAIL{bcolors.ENDC}"

    print(f"Result: {Result}\n\n")


def main():
    """
    Main function
    """
    # Run the client:
    loop = asyncio.get_event_loop()
    #loop.set_debug(True)
    loop.run_until_complete(TaskRunClient(loop))
    loop.close()

    while True:
        asyncio.sleep(1)

if __name__ == "__main__":
    main()
