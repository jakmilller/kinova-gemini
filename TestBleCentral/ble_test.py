import asyncio
import sys
import traceback
from ble_link import BleCentralLink, dbg_ble

def on_instruction(text):
    print(f'\n>>> INSTRUCTION FROM PHONE: "{text}\n', flush = True)

async def heartbeat(link):
    n = 0
    while True:
        await asyncio.sleep(15)
        try:
            await link.send_status(f"Test kit status heartbeat {n}")
            n += 1
        except Exception:
            dbg_ble("heartbeat faile: \n" + traceback.format_exc())

async def main():

    link = BleCentralLink(on_instruction, device_name="RC Phone")
    hb_task = asyncio.create_task(heartbeat(link))
    try:
        await link.run()
    finally:
        hb_task.cancel()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nStopped.")