import asyncio
import websockets

async def test_webots_websocket():
    uri = "ws://localhost:22001"
    async with websockets.connect(uri) as websocket:
        print("✅ Успешное подключение к Webots WebSocket!")
        while True:
            response = await websocket.recv()
            print("📩 Получено сообщение:", response)

asyncio.run(test_webots_websocket())
