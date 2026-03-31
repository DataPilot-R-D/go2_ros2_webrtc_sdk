# Copyright (c) 2024, RoboVerse community
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# WEB_RTC_CON WAS ORIGINALY FORKED from https://github.com/tfoldi/go2-webrtc/tree/master and https://github.com/legion1581/go2_webrtc_connect
# Big thanks for your passion! @tfoldi (Földi Tamás) and @legion1581 (The RoboVerse Discord Group)


import asyncio
import binascii
import uuid
import base64
import hashlib
import json
import logging
import struct
import time
from Crypto.PublicKey import RSA
from Crypto.Cipher import AES
from Crypto.Cipher import PKCS1_v1_5
from cryptography.hazmat.primitives.ciphers.aead import AESGCM
import requests
from aiortc import RTCPeerConnection, RTCSessionDescription, RTCConfiguration


try:
    from scripts_go2.go2_lidar_decoder import LidarDecoder
    decoder = LidarDecoder()
except Exception as e:
    try:
        from unitree_webrtc_connect.lidar.lidar_decoder_libvoxel import LidarDecoder as LibVoxelDecoder
        decoder = LibVoxelDecoder()
        logging.info("Using unitree_webrtc_connect LibVoxelDecoder for lidar")
    except Exception as e2:
        decoder = None
        logging.warning(f"No lidar decoder available ({e}; {e2})")


logging.basicConfig(level=logging.WARN)
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


def calc_local_path_ending(data1):
    # Initialize an array of strings
    strArr = ["A", "B", "C", "D", "E", "F", "G", "H", "I", "J"]

    # Extract the last 10 characters of data1
    last_10_chars = data1[-10:]

    # Split the last 10 characters into chunks of size 2
    chunked = [last_10_chars[i:i + 2] for i in range(0, len(last_10_chars), 2)]

    # Initialize an empty list to store indices
    arrayList = []

    # Iterate over the chunks and find the index of the second character in strArr
    for chunk in chunked:
        if len(chunk) > 1:
            second_char = chunk[1]
            try:
                index = strArr.index(second_char)
                arrayList.append(index)
            except ValueError:
                # Handle case where the character is not found in strArr
                print(f"Character {second_char} not found in strArr.")

    # Convert arrayList to a string without separators
    joinToString = ''.join(map(str, arrayList))

    return joinToString


def generate_aes_key() -> str:
    uuid_32 = uuid.uuid4().bytes
    uuid_32_hex_string = binascii.hexlify(uuid_32).decode('utf-8')
    return uuid_32_hex_string


def decrypt_con_notify_data(encrypted_b64: str) -> str:
    """Decrypt RSA key from firmware version >=1.1.8 using AES-GCM."""
    key = bytes([232, 86, 130, 189, 22, 84, 155, 0, 142, 4, 166, 104, 43, 179, 235, 227])
    data = base64.b64decode(encrypted_b64)
    if len(data) < 28:
        raise ValueError("Decryption failed: input data too short")
    tag = data[-16:]
    nonce = data[-28:-16]
    ciphertext = data[:-28]

    aesgcm = AESGCM(key)
    plaintext = aesgcm.decrypt(nonce, ciphertext + tag, None)
    return plaintext.decode('utf-8')


def rsa_load_public_key(pem_data: str) -> RSA.RsaKey:
    """Load an RSA public key from a PEM-formatted string."""
    key_bytes = base64.b64decode(pem_data)
    return RSA.import_key(key_bytes)


def pad(data: str) -> bytes:
    """Pad data to be a multiple of 16 bytes (AES block size)."""
    block_size = AES.block_size
    padding = block_size - len(data) % block_size
    padded_data = data + chr(padding) * padding
    return padded_data.encode('utf-8')


def aes_encrypt(data: str, key: str) -> str:
    """Encrypt the given data using AES (ECB mode with PKCS5 padding)."""
    # Ensure key is 32 bytes for AES-256
    key_bytes = key.encode('utf-8')

    # Pad the data to ensure it is a multiple of block size
    padded_data = pad(data)

    # Create AES cipher in ECB mode
    cipher = AES.new(key_bytes, AES.MODE_ECB)

    encrypted_data = cipher.encrypt(padded_data)
    encoded_encrypted_data = base64.b64encode(encrypted_data).decode('utf-8')

    return encoded_encrypted_data


def rsa_encrypt(data: str, public_key: RSA.RsaKey) -> str:
    """Encrypt data using RSA and a given public key."""
    cipher = PKCS1_v1_5.new(public_key)

    # Maximum chunk size for encryption with RSA/ECB/PKCS1Padding is key size - 11 bytes
    max_chunk_size = public_key.size_in_bytes() - 11
    data_bytes = data.encode('utf-8')

    encrypted_bytes = bytearray()
    for i in range(0, len(data_bytes), max_chunk_size):
        chunk = data_bytes[i:i + max_chunk_size]
        encrypted_chunk = cipher.encrypt(chunk)
        encrypted_bytes.extend(encrypted_chunk)

    # Base64 encode the final encrypted data
    encoded_encrypted_data = base64.b64encode(encrypted_bytes).decode('utf-8')
    return encoded_encrypted_data


def unpad(data: bytes) -> str:
    """Remove padding from data."""
    padding = data[-1]
    return data[:-padding].decode('utf-8')


def aes_decrypt(encrypted_data: str, key: str) -> str:
    """Decrypt the given data using AES (ECB mode with PKCS5 padding)."""
    # Ensure key is 32 bytes for AES-256
    key_bytes = key.encode('utf-8')

    # Decode Base64 encrypted data
    encrypted_data_bytes = base64.b64decode(encrypted_data)

    # Create AES cipher in ECB mode
    cipher = AES.new(key_bytes, AES.MODE_ECB)

    # Decrypt data
    decrypted_padded_data = cipher.decrypt(encrypted_data_bytes)

    # Unpad the decrypted data
    decrypted_data = unpad(decrypted_padded_data)

    return decrypted_data


def make_local_request(path, body=None, headers=None):
    try:
        # Send POST request with provided path, body, and headers
        response = requests.post(url=path, data=body, headers=headers)

        # Check if the request was successful (status code 200)
        response.raise_for_status()  # Raises an HTTPError for bad responses (4xx, 5xx)

        if response.status_code == 200:
            return response  # Returning the whole response object if needed
        else:
            # Handle non-200 responses
            return None

    except requests.exceptions.RequestException as e:
        # Handle any exception related to the request (e.g., connection errors, timeouts)
        logging.error(f"An error occurred: {e}")
        return None


class Go2Connection():
    def __init__(
            self,
            robot_ip=None,
            robot_num=None,
            token="",
            on_validated=None,
            on_message=None,
            on_open=None,
            on_video_frame=None,
    ):

        # Store params — PC and data channel created in connect() to match
        # unitree-webrtc-connect's init_webrtc() pattern
        self.pc = None
        self.data_channel = None
        self.robot_ip = robot_ip
        self.robot_num = str(robot_num)
        self.token = token
        self.on_validated = on_validated
        self.on_message = on_message
        self.on_open = on_open
        self.on_video_frame = on_video_frame

        # Data channel state (ported from unitree-webrtc-connect)
        self.data_channel_opened = False
        self.validation_key = ""
        self.heartbeat_timer = None

    def on_connection_state_change(self):
        logger.info(f"Connection state is {self.pc.connectionState}")

    async def on_track(self, track):
        logger.info(f"Receiving {track.kind}")
        if track.kind == "audio":
            pass
        elif track.kind == "video":
            frame = await track.recv()
            logger.info(f"Received frame {frame}")
            if self.on_video_frame:
                await self.on_video_frame(track, int(self.robot_num))

    def on_data_channel_open(self):
        logger.info("Data channel on('open') fired")
        if self.on_open:
            self.on_open()

    async def on_data_channel_message(self, msg):
        """Handle incoming data channel messages with proper type routing (async)."""

        # Force data channel to open state on first message (Go2 protocol)
        if self.data_channel.readyState != "open":
            self.data_channel._setReadyState("open")

        try:
            if isinstance(msg, str):
                msgobj = json.loads(msg)
                msg_type = msgobj.get("type", "")

                if msg_type == "validation":
                    await self.handle_validation(msgobj)
                elif msg_type == "err":
                    self.handle_err(msgobj)
                elif msg_type == "heartbeat":
                    pass  # Heartbeat ACK from robot, no action needed
                elif msg_type == "rtc_inner_req":
                    self.handle_rtc_inner_req(msgobj)

            elif isinstance(msg, bytes):
                msgobj = Go2Connection.deal_array_buffer(msg)

            if self.on_message:
                self.on_message(msg, msgobj, self.robot_num)

        except json.JSONDecodeError:
            pass
        except Exception as e:
            logger.warning(f"Error handling data channel message: {e}")

    # --- Validation (ported from unitree-webrtc-connect/msgs/validation.py) ---

    async def handle_validation(self, message):
        """Handle validation challenge/response from robot (async)."""
        if message.get("data") == "Validation Ok.":
            logger.info("Validation succeeded!")
            self.data_channel_opened = True

            # Start heartbeat to keep connection alive
            self.start_heartbeat()

            # Enable video channel
            self._send_raw("vid", "", "on")

            if self.on_validated:
                self.on_validated(self.robot_num)
        else:
            # Validation challenge - respond with encrypted key
            self.validation_key = message.get("data", "")
            logger.info("Received validation challenge, sending response...")

            # Force channel open so we can send the response
            if self.data_channel.readyState != "open":
                self.data_channel._setReadyState("open")

            self._send_raw("validation", "", self.encrypt_key(self.validation_key))
            # Yield to let aiortc process the outgoing message
            await asyncio.sleep(0)

    def handle_err(self, message):
        """Handle error messages - re-validate if needed."""
        if message.get("info") == "Validation Needed.":
            logger.info("Re-validation requested, sending key...")
            self._send_raw("validation", "", self.encrypt_key(self.validation_key))

    # --- Heartbeat (ported from unitree-webrtc-connect/msgs/heartbeat.py) ---

    def start_heartbeat(self):
        """Start sending heartbeat every 2 seconds."""
        loop = asyncio.get_event_loop()
        self.heartbeat_timer = loop.call_later(2, self.send_heartbeat)
        logger.info("Heartbeat started")

    def stop_heartbeat(self):
        """Stop heartbeat timer."""
        if self.heartbeat_timer:
            self.heartbeat_timer.cancel()
            self.heartbeat_timer = None

    def send_heartbeat(self):
        """Send a heartbeat message and reschedule."""
        if self.data_channel.readyState == "open":
            current_time = time.time()
            formatted_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(current_time))
            self._send_raw("heartbeat", "", {
                "timeInStr": formatted_time,
                "timeInNum": int(current_time),
            })
        # Reschedule
        loop = asyncio.get_event_loop()
        self.heartbeat_timer = loop.call_later(2, self.send_heartbeat)

    # --- RTC Inner Request (ported from unitree-webrtc-connect/msgs/rtc_inner_req.py) ---

    def handle_rtc_inner_req(self, message):
        """Handle RTT probe requests from robot - echo them back."""
        info = message.get("info")
        if info and info.get("req_type") == "rtt_probe_send_from_mechine":
            self._send_raw("rtc_inner_req", "", info)

    # --- Send helpers ---

    def _send_raw(self, msg_type, topic, data):
        """Send a message directly on the data channel, bypassing readyState checks."""
        payload = json.dumps({"type": msg_type, "topic": topic, "data": data})
        self.data_channel.send(payload)

    def publish(self, topic, data, msg_type):
        """Send a message on the data channel (with open check)."""
        if not self.data_channel_opened and self.data_channel.readyState != "open":
            logger.info(f"Data channel is not open. State is {self.data_channel.readyState}")
            return

        payload = json.dumps({"type": msg_type, "topic": topic, "data": data})
        self.data_channel.send(payload)

    # --- Wait for data channel (ported from unitree-webrtc-connect) ---

    async def wait_datachannel_open(self, timeout=15):
        """Wait for the data channel validation to complete."""
        try:
            await asyncio.wait_for(self._wait_for_open(), timeout)
            logger.info("Data channel is validated and ready!")
        except asyncio.TimeoutError:
            logger.error(f"Data channel did not validate within {timeout}s")

    async def _wait_for_open(self):
        """Poll until data_channel_opened is set by validation."""
        while not self.data_channel_opened:
            await asyncio.sleep(0.1)

    # --- Connection ---

    async def connect(self):
        # Use the proven unitree_webrtc_connect library for the full
        # WebRTC + data channel + validation handshake.
        from unitree_webrtc_connect.webrtc_driver import UnitreeWebRTCConnection
        from unitree_webrtc_connect.constants import WebRTCConnectionMethod
        from unitree_webrtc_connect.webrtc_video import WebRTCVideoChannel

        # Register our video callback BEFORE connect() so it's called by
        # the library's track_handler when the video track arrives.
        _original_video_init = WebRTCVideoChannel.__init__
        _on_video = self.on_video_frame
        _rnum = int(self.robot_num)

        def _patched_video_init(video_self, pc, datachannel):
            _original_video_init(video_self, pc, datachannel)
            if _on_video:
                video_self.add_track_callback(
                    lambda track: _on_video(track, _rnum)
                )

        WebRTCVideoChannel.__init__ = _patched_video_init

        self._uwc = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip=self.robot_ip)

        # Patch: increase DC timeout from 5s to 15s, don't sys.exit
        from unitree_webrtc_connect.webrtc_datachannel import WebRTCDataChannel
        _orig_wait = WebRTCDataChannel.wait_datachannel_open
        async def _wait(dc_self, timeout=15):
            try:
                await asyncio.wait_for(dc_self._wait_for_open(), timeout)
            except asyncio.TimeoutError:
                logger.warning("DC validation timeout %ds — continuing", timeout)
        WebRTCDataChannel.wait_datachannel_open = _wait

        # Patch: prevent sys.exit(1) on SDP/answer failure from killing ROS2.
        import sys
        _orig_exit = sys.exit
        _sdp_error = None
        def _no_exit(code=0):
            nonlocal _sdp_error
            _sdp_error = f"SDP exchange failed (exit code {code})"
            raise ConnectionError(_sdp_error)
        sys.exit = _no_exit

        try:
            await self._uwc.connect()
        except ConnectionError:
            logger.error(_sdp_error)
        finally:
            sys.exit = _orig_exit
            WebRTCDataChannel.wait_datachannel_open = _orig_wait
            WebRTCVideoChannel.__init__ = _original_video_init

        # Grab the working PC and data channel
        self.pc = self._uwc.pc
        self.data_channel = self._uwc.datachannel.channel
        self.data_channel_opened = self._uwc.datachannel.data_channel_opened

        # Register our message handler on the working data channel
        @self.data_channel.on("message")
        async def on_dc_message(msg):
            await self.on_data_channel_message(msg)

        @self.pc.on("connectionstatechange")
        def on_conn_change():
            self.on_connection_state_change()

        # Start video frame processing as a background task.
        # The library's on_track handler consumes the first frame, then
        # calls track_handler which may or may not invoke our callback
        # (depends on timing). As a fallback, we also get the video
        # receiver's track directly and start our own recv loop.
        if self.on_video_frame:
            robot_num = int(self.robot_num)
            on_video = self.on_video_frame

            async def _start_video():
                # Wait briefly for the library's on_track to consume its frame
                await asyncio.sleep(0.5)
                for receiver in self.pc.getReceivers():
                    if receiver.track and receiver.track.kind == "video":
                        logger.info(f"Starting video recv loop for robot {self.robot_num}")
                        await on_video(receiver.track, robot_num)
                        return
                logger.warning("No video track found on receivers")

            asyncio.ensure_future(_start_video())

        logger.info(f"Connected via unitree_webrtc_connect, DC opened={self.data_channel_opened}")

    # --- Static helpers ---

    @staticmethod
    def hex_to_base64(hex_str):
        bytes_array = bytes.fromhex(hex_str)
        return base64.b64encode(bytes_array).decode("utf-8")

    @staticmethod
    def encrypt_key(key):
        prefixed_key = f"UnitreeGo2_{key}"
        encrypted = Go2Connection.encrypt_by_md5(prefixed_key)
        return Go2Connection.hex_to_base64(encrypted)

    @staticmethod
    def encrypt_by_md5(input_str):
        hash_obj = hashlib.md5()
        hash_obj.update(input_str.encode("utf-8"))
        return hash_obj.hexdigest()

    @staticmethod
    def deal_array_buffer(n):
        if isinstance(n, bytes):
            # Check for lidar format (header 2, 0)
            header_1, header_2 = struct.unpack_from('<HH', n, 0)
            if header_1 == 2 and header_2 == 0:
                # Lidar format
                buf = n[4:]
                length = struct.unpack_from('<I', buf, 0)[0]
                json_data = buf[8:8 + length]
                binary_data = buf[8 + length:]
                obj = json.loads(json_data.decode('utf-8'))
                decoded_data = decoder.decode(binary_data, obj['data'])
                obj['data']['data'] = decoded_data
                return obj
            else:
                # Normal format
                length = struct.unpack("H", n[:2])[0]
                json_segment = n[4: 4 + length]
                compressed_data = n[4 + length:]
                json_str = json_segment.decode("utf-8")
                obj = json.loads(json_str)
                decoded_data = decoder.decode(compressed_data, obj['data'])
                obj["decoded_data"] = decoded_data
                return obj
        return None
