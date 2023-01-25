import threading
from abc import abstractproperty, ABC, abstractmethod
from typing import Callable, Any, Union


class TransceiverBase(ABC):
    """
    Abstract implementation of a serial transceiver that sends and received packetized data
    The data packets are formatted in the format: header + data_length + data_body + checksum
    """
    lock = threading.Lock()
    header = bytearray(b'\x69\x69\x20\x20')

    @property
    @abstractmethod
    def socket(self):
        pass

    def __init__(self, recv_callback: Callable[[bytearray], Any]):
        self.listener = None
        self.callback = recv_callback

    @abstractmethod
    def recv(self, bytes_to_read: int) -> bytearray:
        """Receives `bytes_to_read` bytes."""
        pass

    def read_packet(self):
        buffer = bytearray()
        while True:
            buf_ptr = 0

            # Recv header
            buffer += self.recv(4)
            if not buffer:
                break

            start_loc = buffer.find(self.header)
            if start_loc == -1:
                continue
            else:
                buffer = buffer[start_loc:]

            if buffer[0:4] != self.header:
                continue

            # Read header (move past it)
            buffer += self.recv(4)
            buf_ptr += 4
            # Read data length as u32
            data_length = int.from_bytes(buffer[buf_ptr:buf_ptr + 4], 'big', signed=False)
            buf_ptr += 4

            # Recv data body and checksum (which is two bytes)
            buffer += self.recv(data_length + 2)

            # Read data (move past it, ignore until checksum is verified)
            # Record where the data starts so we don't have to recalculate it later
            data_start = buf_ptr
            # This is where the data ends (since we don't have to increment the pointer anymore)
            buf_ptr += data_length
            data_end = buf_ptr

            # Read checksum
            checksum = int.from_bytes(buffer[buf_ptr:buf_ptr + 2], 'big', signed=False)

            # Calculate own checksum
            own_checksum = ~(sum(buffer[:-2])) & 0xFFFF

            # Verify checksum and only perform callback if it matches
            if checksum == own_checksum:
                # Extract the data
                packet = buffer[data_start:data_end]
                self.callback(packet)
                break

    @abstractmethod
    def connect(self) -> bool:
        """Opens the connection, returning a success state. Call `start_recv` after this to start the receiver."""
        pass

    @abstractmethod
    def send(self, data: Union[bytearray, str]):
        pass

    def __del__(self):
        del self.listener
