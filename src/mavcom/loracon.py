import json
import time
import serial
import socket
import threading
from datetime import datetime

class RadioNotFound(Exception):
    def __init__(self, message: str) -> None:
        self.message = message

    def __str__(self) -> str:
        return f"No Lora radio found: {self.message}"
    
class RecipientNotAck(Exception):
    # def __init__(self, *args: object) -> None:
    #     super().__init__(*args)

    def __str__(self) -> str:
        return f"No acknowledgement from ground station"

class UnknownTxMode(Exception):
    def __init__(self, mode, *args: object) -> None:
        super().__init__(*args)
        self.mode = mode

    def __str__(self) -> str:
        return f"Unknown transmit mode in received message: {self.mode}"


class LoraController:
    """
    Lora messages will be transmitted as JSON objects in plain text.

    Structured as this:

    `{"tm": "tx mode", "u": "recipient name (if not to gs)", "f": "this (sender) unit's name, "m": "message body", "t": time (H%M%S%f)}`

    A field `sr` is added if it is a statrep, to allow the groundstation to easily parse.
    All transmissions have a timestamp (h-m-s-ms) allowing units to identify if they have already rebroadcasted a broadcast request

    Recipient/tx mode can be:
    dg - direct groundstation
    du - direct to specific unit
    bg - broadcast toward ground station
    bu - broadcast toward specific unit
    """

    def __init__(self, port="/dev/ttyS1") -> None:
        self.awaiting_ack = False
        self.awaiting_ack = False
        self.ack_timeout = 8
        self.port = port

        # radio is half duplex; we need to stop the listener if we are transmitting
        self.transmitting = False

        try:
            self.radio = serial.Serial(port=f'/dev/{self.port}',baudrate = 9600,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,bytesize=serial.EIGHTBITS,timeout=1)
            self.listener = threading.Thread(target=self.receive)
            print(f"MAVCOM: Lora connected on {self.port}")
            
        except Exception as err:
            print(f"MAVCOM: Lora error: {err}")
            self.raise_error(err=err, err_type=RadioNotFound)
        
    def raise_error(self, err: str, err_type: Exception) -> Exception:
        return err_type(message=err)
        
    def start(self):
        self.listener.start()
        print("MAVCOM: Lora radio started")

    def broadcast(self, msg_dict):
        """
        Broadcast a message to be forwarded by any units in range, to try and
        reach the ground station.

        Listens for an ack, and reports back if no ack. This terminates efforts to transmit
        this message.
        """
        print("MAVCOM: Lora message - ", msg_dict)
        if self.check_rebroadcast(msg_dict):
            self.transmit(msg=msg_dict, broadcast=True)
            self.set_lora_msg_to_rebroadcast(timestamp=msg_dict["t"])
        else:
            print("MAVCOM: Already rebroadcasted this")
            
    def check_rebroadcast(self, msg):
        """Returns True if the message should be rebroadcasted"""
        message_archive = self.get_lora_messages()
        stored_message_timestamps = [m for m in message_archive.keys()]
        
        for msgid, stored_msg in message_archive.items():
            if msgid == stored_msg[0] and stored_msg[3] == "no":
                return True # rebroadcast this message, as I haven't already
            
        return False

    def transmit(self, msg, ack_to=None, send_ack=False, broadcast=False):
        """
        Message transmission
        """
        ack = {
            "tm": "du",
            "u": f"{ack_to}",
            "f": f"{socket.gethostname()}",
            "m": "lora_ack",
            "t": datetime.now().strftime("%H%M%S%f")
            }
        
        if send_ack: msg = ack
        
        self.transmitting = True
        print("MAVCOM: Lora transmit...")
        msg = self.convert_message(msg)
        if send_ack:
            encoded_msg = bytes(msg,'utf-8')
            s = self.radio.write(encoded_msg)
            self.transmitting = False
            print(f"MAVCOM: Lora - transmitter {socket.gethostname()}: sent back ack")
            return True
        else:
            encoded_msg = bytes(msg,'utf-8')
            s = self.radio.write(encoded_msg)
            print(f"MAVCOM: Lora - transmitter {socket.gethostname()}: sent message")
            self.transmitting = False
            self.awaiting_ack = True

            counter = self.ack_timeout
            while self.awaiting_ack:
                print("MAVCOM: Lora - Waiting for Ack from recipient...")
                time.sleep(1)

                if counter == 0:
                    print("MAVCOM: Lora - No ack from recipient")
                    print(
                        "MAVCOM: Lora - On receiving the following exception sender/signaller upstream function should now call lora broadcast function, if mesh mode is True"
                    )
                    self.awaiting_ack = False
                    raise RecipientNotAck

                counter -= 1
            rcvd_str = msg.decode(encoding="utf-8")
            rcvd_msg = json.loads(rcvd_str)
            print(f"MAVCOM: Lora - transmitter {socket.gethostname()}: ack received, reactivate listening...")
            
            return True

    def receive(self, ack_to=None, received_data=None):
        """
        Run as thread, constantly listening for messages from GS or
        other units.

        received_data as a param is only for testing. In reality it will be
        listening with `received_data = self.radio.readline()` inside the while loop.

        Decodes the message, converts the string to json/dict, and parses.
        """
        message_processed = False
        time.sleep(1)
        print("MAVCOM: Lora listening")

        while True:
            message_processed = False
            received_data = self.radio.readline()
            if not self.transmitting and received_data and not message_processed:
                print(f"MAVCOM: Lora - receiver {socket.gethostname()}: received something: {received_data}")
                # message_processed = False
                try:

                    rcvd_str = received_data.decode(encoding="utf-8")
                    # rcvd_str = received_data.encode(encoding="utf-8")

                    rcvd_msg = json.loads(rcvd_str)
                    # print(rcvd_msg)

                    # if rcvd_msg["u"] == socket.gethostname():
                    if rcvd_msg["u"] == socket.gethostname():
                        print("MAVCOM: Lora - this message is for ME")
                        print(rcvd_msg)
                        message_processed = True
                        self.store_lora_msg(message=rcvd_msg)
                        
                    elif rcvd_msg["t"] == "bu":
                        print(
                            f"MAVCOM: Lora - this message is for {rcvd_msg['u']}, I need to re-broadcast it"
                        )
                        self.broadcast(msg_dict=rcvd_msg)
                        self.store_lora_msg(message=rcvd_msg, rebroadcasted="yes")
                        message_processed = True
                        
                    elif rcvd_msg["t"] == "bg":
                        print(
                            f"MAVCOM: Lora - I need to re-broadcast this message from {rcvd_msg['f']} to {rcvd_msg['u']}"
                        )
                        self.broadcast(msg_dict=rcvd_msg)
                        self.store_lora_msg(message=rcvd_msg, rebroadcasted="yes")
                        message_processed = True
                        
                    elif rcvd_msg["t"] == "dg" or rcvd_msg["t"] == "du":
                        print(
                            f"MAVCOM: Lora - message heard from {rcvd_msg['f']}, to {rcvd_msg['u']} - {rcvd_msg['m']}"
                        )
                        
                    else:
                        message_processed = True
                        raise UnknownTxMode(rcvd_msg["t"])
                except UnknownTxMode as utx:
                    message_processed = True
                    print("MAVCOM: Lora - ", utx)
                finally:
                    # if unit received a message that isn't just an ack, or receives a broadcast meant for ALL units, send an ack
                    if rcvd_msg["m"] != "lora_ack" or (rcvd_msg["t"] == "bu" and rcvd_msg["u"] == "all"):
                        self.transmit(msg=None, ack_to=rcvd_msg['f'], send_ack=True)
                        print(f"MAVCOM: Lora - listener {socket.gethostname()} sent ack")
                    else:
                        self.awaiting_ack = False
                        print(f"MAVCOM: Lora - listener {socket.gethostname()}: ack received from {rcvd_msg['u']}")
                        
                    

    def convert_message(self, msg, statrep: bool = False, timestamp: str = datetime.now().strftime("%H%M%S%f")):
        """
        Converts a message to string for lora transmit.
        i.e. statrep (JSON) convert to text.

        If its a statrep, add a statrep field so the ground station can parse it.
        
        If the upstream function has only provided a string as the message, put it into a dict,
        convert to JSON string and send it to the groundstation.
        
        Timestamp should be provided by the upstream function. This serves as a unique identifier for the message that
        allows units to check it against their stored messages and mark as rebroadcast, to avoid infinite loops of
        rebroadcasting.
        """
        if not type(msg) == dict:
            msg = {
                "tm": "dg",
                "u": "g",
                "f": f"{socket.gethostname()}",
                "m": f"{str(msg)}",
                "t": timestamp
            }
        if statrep:
            msg["sr"] = 1
        msg_string = json.dumps(msg)
        return msg_string
    
    def store_lora_msg(message: dict, rebroadcasted: str = "no") -> None:
        with open("messages.json", "r") as msg_file:
            messages = json.load(msg_file)
        
        newmsg = {
            "time": message['t'],
            "message": message['m'],
            "to": message['u'],
            "rebroadcasted": rebroadcasted
        }
        messages.append(newmsg)
        
        with open("messages.json", "w") as msg_file:
            json.dump(messages, msg_file)
            
    def get_lora_messages() -> dict:
        with open("messages.json", "r") as msg_file:
            messages = json.load(msg_file)
        return messages
    
    def set_lora_msg_to_rebroadcast(timestamp: str):
        with open("messages.json", "r") as msg_file:
            messages = json.load(msg_file)
        for m in messages:
            if m['time'] == timestamp:
                m['rebroadcasted'] = "yes"
        
if __name__ == "__main__":
    lora = LoraController()
    lora.start()