from bleak import BleakScanner, BleakClient
import traceback
import asyncio
from .ble_framing import chunk_status, split_instructions

SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
INSTRUCTION_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
STATUS_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
SCAN_TIMEOUT = 30.0
RECONNECT_DELAY = 3.0

def dbg_ble(msg):
    print(f"[BLE] {msg}", flush = True)

class BleCentralLink:

    """Scans for the phone and connects"""

    def __init__(self, on_instruction, service_uuid = SERVICE_UUID, device_name = None):

        self._on_instruction = on_instruction
        self._service_uuid = service_uuid
        self._device_name = device_name
        self._client = None
        self._inbuf = bytearray()
        self._running = False

    # async def _find_phone(self):
        # devices = await BleakScanner.discover(timeout = SCAN_TIMEOUT, return_adv = True)
        # name_match = None
        # uuid_match = None

        # for addr, (dev, adv) in devices.items():
        #     uuids = [u.lower() for u in (adv.service_uuids or [])]
        #     name = dev.name or adv.local_name or "(no name)"
        #     dbg_ble(f"  {addr} rssi = {adv.rssi:>4} name = {name!r} uuids = {uuids}")

        #     if self._service_uuid.lower() in uuids: 
        #         uuid_match = dev
        #     elif self._device_name and self._device_name.lower() in name.lower():
        #         name_match = dev

        # match = uuid_match or name_match

        # if match is None:
        #     dbg_ble("No matching device. Is the phone app open and in the foreground? Is it advertising the right uuid? is bluetooth on for the phone and the computer?")

        # return match

    async def _find_phone(self):
            seen = set()
    
            def _match(dev, adv):
                uuids = [u.lower() for u in (adv.service_uuids or [])]
                name = dev.name or adv.local_name or "(no name)"
    
                if dev.address not in seen:
                    seen.add(dev.address)
                    dbg_ble(f"{dev.address} name={name!r} uuids={uuids}")
    
                if self._service_uuid.lower() in uuids:
                    return True
    
                if self._device_name and self._device_name.lower() in name.lower():
                    return True
    
                return False
            
            device = await BleakScanner.find_device_by_filter(_match, timeout=SCAN_TIMEOUT)
    
            if device is None:
                dbg_ble("No matching device. Is the phone app open and in the foreground? Is it advertising the right uuid? is bluetooth on for the phone and the computer?")
    
            return device

    def _on_notify(self, _char, data: bytearray):
        try:
            dbg_ble(f"Notify {len(data)} bytes, raw = {bytes(data)!r}")
            self._inbuf.extend(bytes(data))

            instrs, self._inbuf = split_instructions(self._inbuf)

            if not instrs and self._inbuf:
                text = bytes(self._inbuf).decode("utf-8", errors = "replace").strip()
                if text:
                    dbg_ble("no newline, entire payload is one instruction")
                    instrs = [text]
                    self._inbuf = bytearray()

            for text in instrs:
                dbg_ble(f'Parsed instruction: "{text}"')
                self._on_instruction(text)

        except Exception:
            dbg_ble("Exception in notify handler: \n" + traceback.format_exc())
            

    def _on_disconnect(self, _client):
        dbg_ble("Phone disconnected")
        self._client = None

    async def send_status(self, text):
        if self._client is None or not self._client.is_connected:
            dbg_ble(f'Cannot send status (not connected): "{text}"')
            return
        try:
            frames = chunk_status(text)
            dbg_ble(f"Sending status ({len(text)} chars) as {len(frames)} frames")
            for i, f in enumerate(frames, 1):
                dbg_ble(f"  write frame {i}/{len(frames)} ({len(f)} bytes)")
                await self._client.write_gatt_char(STATUS_UUID, f, response = False)
        except Exception:
            dbg_ble("Exception sending status: \n" + traceback.format_exc())

    async def run(self):
        self._running = True
        while self._running:
            try:
                device = await self._find_phone()
                if device is None:
                    dbg_ble(f"Retrying to find phone...")
                    await asyncio.sleep(RECONNECT_DELAY)
                    continue

                dbg_ble(f"Connecting to {device.address}...")
                async with BleakClient(device, disconnected_callback = self._on_disconnect) as client:
                    self._client = client
                    dbg_ble("Connected")
                    dbg_ble("services & characteristics found: ")
                    found_service = found_instr = found_status = False
                    for svc in client.services:
                        dbg_ble(f"   service {svc.uuid}")
                        if svc.uuid.lower() == self._service_uuid.lower():
                            found_service = True
                        for ch in svc.characteristics:
                            dbg_ble(f"      char {ch.uuid} props = {ch.properties}")
                            if ch.uuid.lower() == INSTRUCTION_UUID.lower():
                                found_instr = True
                            if ch.uuid.lower() == STATUS_UUID.lower():
                                found_status = True

                    if not found_service:
                        dbg_ble(f"Warning: expected service {self._service_uuid} not found!")
                    if not found_instr:
                        dbg_ble(f"Warning: expected characteristic {INSTRUCTION_UUID} not found!")
                    if not found_status:
                        dbg_ble(f"Warning: expected characteristic {STATUS_UUID} not found!")

                    dbg_ble(f"Subscribing to {INSTRUCTION_UUID} ...")
                    await client.start_notify(INSTRUCTION_UUID, self._on_notify)
                    dbg_ble("Subscribed. Waiting for instructions from the phone")
                    self._fail_count = 0

                    import time as _time
                    t0 = _time.monotonic

                    while client.is_connected and self._running:
                        await asyncio.sleep(1.0)

                    uptime = _time.monotonic - t0
                    dbg_ble(f"Link dropped after {uptime:.1f}s connected")

                dbg_ble("Connection closed")

            except Exception:
                dbg_ble("Exception in ble loop: \n" + traceback.format_exc())
                dbg_ble("Common causes: phone app not advertising, wrong characteristic uuid, phone bluetooth off, or the phone dropped the link")

            finally:
                self._client = None

            if self._running:
                dbg_ble("reconnecting...")
                self._fail_count = getattr(self, "_fail_count", 0) + 1
                dbg_ble("attempt {self._fail_count}")
                await asyncio.sleep(RECONNECT_DELAY)

    def stop(self):
        self._running = False