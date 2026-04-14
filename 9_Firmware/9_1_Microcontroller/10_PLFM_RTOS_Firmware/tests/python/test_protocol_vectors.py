"""Host-side protocol vector checks for PLFM frame format."""

from dataclasses import dataclass


SOF = 0xA5
EOF = 0x5A


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


@dataclass
class Frame:
    cmd: int
    seq: int
    payload: bytes

    def encode(self) -> bytes:
        header = bytes(
            [
                SOF,
                self.cmd & 0xFF,
                len(self.payload) & 0xFF,
                (len(self.payload) >> 8) & 0xFF,
                self.seq & 0xFF,
                (self.seq >> 8) & 0xFF,
                (self.seq >> 16) & 0xFF,
                (self.seq >> 24) & 0xFF,
            ]
        )
        crc = crc16_ccitt(header[1:] + self.payload)
        return header + self.payload + bytes([crc & 0xFF, (crc >> 8) & 0xFF, EOF])


def test_encode_get_config_shape() -> None:
    data = Frame(cmd=0x01, seq=1, payload=b"").encode()
    assert data[0] == SOF
    assert data[-1] == EOF
    assert len(data) == 11


def test_crc_catches_single_bit_flip() -> None:
    data = bytearray(Frame(cmd=0x90, seq=7, payload=b"\x01\x02\x03\x04").encode())
    good_crc = int.from_bytes(data[-3:-1], "little")
    data[8] ^= 0x01
    bad_crc = crc16_ccitt(bytes(data[1:-3]))
    assert good_crc != bad_crc
