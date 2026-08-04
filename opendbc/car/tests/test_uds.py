from opendbc.car.uds import IsoTpMessage


class FakeCanClient:
  sub_addr = None
  tx_addr = 0x745
  rx_addr = 0x74D

  def __init__(self):
    self.frames: list[bytes] = []

  def recv(self, drain: bool = False):
    if drain:
      self.frames.clear()
      return iter(())
    frames, self.frames = self.frames, []
    return iter(frames)

  def send(self, msgs: list[bytes], delay: float = 0) -> None:
    pass


def test_isotp_ignores_reserved_frame_type_bus_traffic():
  client = FakeCanClient()
  msg = IsoTpMessage(client)
  msg.send(b"\x22\xf1\x90")

  client.frames = [b"\x40\x01\x02\x03", b"\x03\x62\xf1\x90"]
  response, in_progress = msg.recv(timeout=0)

  assert response == b"\x62\xf1\x90"
  assert not in_progress
