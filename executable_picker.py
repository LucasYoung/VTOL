'''
The GCS allocates roles to vehicles. The purpose of this program is to connect to the GCS and then
read a start message from the GCS, which contains a certain job_type. It then runs the
corresponding VTOL program (quick scan, detailed search, guide) based on the start message
'''
from autonomy import setup_xbee, bad_msg
from quick_scan import quick_scan
from detailed_search import detailed_search
from util import parse_configs
import sys
import json
import time

xbee = None
gcs_timestamp = 0
connection_timestamp = 0

def xbee_callback(message):
    global gcs_timestamp
    global connection_timestamp

    address = message.remote_device.get_64bit_addr()
    msg = json.loads(message.data)

    try:
        msg_type = msg["type"]

        if msg_type == "connectionAck":
            gcs_timestamp = msg['clocktime']
            connection_timestamp = time.time()
            
        elif msg_type == "start":
            # detach this callback so that the corresponding role can use its own callback
            xbee.del_data_received_callback(xbee_callback)

            job_type = msg['jobType']
            if job_type == "quickScan":
                quick_scan(gcs_timestamp = gcs_timestamp, connection_timestamp = connection_timestamp)
            elif job_type == "detailedSearch":
                detailed_search(gcs_timestamp = gcs_timestamp, connection_timestamp = connection_timestamp)
            elif job_type == "guide":
                # TODO
                pass
            else:
                bad_msg(address, "Unknown jobType: \'" + job_type + "\'")

        else:
            bad_msg(address, "Unknown message type: \'" + msg_type + "\'")

    # KeyError if message was missing an expected key
    except KeyError as e:
        bad_msg(address, "Missing \'" + e.args[0] + "\' key")

def main():
    configs = parse_configs(sys.argv)

    # no comms simulation; that wouldn't be useful as this program is supposed to interact w/ GCS
    global xbee
    xbee = setup_xbee()

    # send connection message
    connection_message = {
        "type": "connect",
        "time": 0, # This field is currently not used
        "sid": configs['vehicle_id'],
        "tid": 0, # The ID of GCS
        "id": 0, # The ID of this message

        "jobsAvailable": ["quickScan", "detailedSearch", "guide"]
    }

    # Instantiate a remote XBee device object to send data.
    send_xbee = RemoteXBeeDevice(xbee, address)
    xbee.send_data(send_xbee, json.dumps(connection_message))

    # wait to receive the connection ack and start message
    xbee.add_data_received_callback(xbee_callback)

if __name__ == "__main__":
    main()
