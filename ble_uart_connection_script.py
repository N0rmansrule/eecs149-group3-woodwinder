import asyncio
import sys
from bleak import BleakScanner, BleakClient

# ===== CONFIG (Matches your ESP32 Code) =====
TARGET_NAME  = "ESP32_BLE_UnoRelay"
NUS_RX_UUID  = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  # Write (to ESP32)
NUS_TX_UUID  = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  # Notify (from ESP32)
# ============================================

def on_notify(sender: int, data: bytearray):
    """Handles incoming messages from the Arduino/ESP32"""
    try:
        text = data.decode("utf-8", errors="replace").strip()
        print(f"\r[ESP32]: {text}")
        print("> ", end="", flush=True)
    except Exception as e:
        print(f"\n[Error decoding]: {e}")

async def run_chat(client: BleakClient):
    """Loop to read user input and send to ESP32"""
    print(f"\n[ble] Connected to {TARGET_NAME}")
    await client.start_notify(NUS_TX_UUID, on_notify)
    print("[ble] Notifications enabled. Ready for commands (1, 0, PAUSE, etc.)")
    print("--- Type and press Enter (Ctrl+C to quit) ---")

    loop = asyncio.get_running_loop()
    while True:
        # Read input from terminal without blocking the async loop
        line = await loop.run_in_executor(None, sys.stdin.readline)
        if not line: break
        
        msg = line.strip()
        if msg:
            # Send message to ESP32
            # response=True ensures it waits for GATT layer ack
            await client.write_gatt_char(NUS_RX_UUID, msg.encode(), response=True)

async def main():
    print(f"[ble] Scanning for {TARGET_NAME}...")
    
    # Auto-find device by name
    device = await BleakScanner.find_device_by_filter(
        lambda d, ad: d.name == TARGET_NAME
    )

    if not device:
        print(f"[error] Could not find {TARGET_NAME}. Is it turned on?")
        return

    print(f"[ble] Found device: {device.address}")
    
    try:
        async with BleakClient(device) as client:
            await run_chat(client)
    except KeyboardInterrupt:
        print("\n[chat] User exited.")
    except Exception as e:
        print(f"\n[ble] Connection error: {e}")

if __name__ == "__main__":
    asyncio.run(main())