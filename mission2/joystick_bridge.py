#!/usr/bin/env python3
"""
joystick_bridge.py — WebSocket ↔ ROS2 köprüsü

HTML arayüzünün ws://localhost:8765 üzerinden gönderdiği komutları
/swarm/joystick_cmd topic'ine yayınlar.

Mesaj türleri:
    {type: "takeoff"}
    {type: "land"}
    {type: "mode_change", mode: "HAREKET"|"MANEVRA"}
    {type: "formation_change", formation: "V"|"OKBASI"|"CIZGI"}
    {type: "key_down", key: "w"|"a"|"s"|...}
    {type: "key_up",   key: "w"|"a"|"s"|...}
    {type: "key_press", key: "v"}   # tek seferlik

Bağımlılık:
    pip3 install websockets --break-system-packages
"""

import asyncio
import json
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
import websockets

WS_HOST = '0.0.0.0'
WS_PORT = 8765


class BridgeNode(Node):
    def __init__(self):
        super().__init__('joystick_bridge')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=10)

        self.cmd_pub = self.create_publisher(
            String, '/swarm/joystick_cmd', qos)

        # Mission -> Web yönlü feedback
        self.ws_clients = set()
        self.create_subscription(
            String, '/swarm/joystick_feedback',
            self._feedback_cb, qos)

        # async loop referansı (thread-safe push için)
        self.ws_loop = None

        self.get_logger().info(
            f'Bridge hazır: ws://{WS_HOST}:{WS_PORT} → /swarm/joystick_cmd')

    def publish_cmd(self, payload):
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.cmd_pub.publish(msg)

    def _feedback_cb(self, msg):
        """Mission'dan gelen mesajı tüm web istemcilerine yolla."""
        if self.ws_loop is None:
            return
        asyncio.run_coroutine_threadsafe(
            self._broadcast(msg.data), self.ws_loop)

    async def _broadcast(self, text):
        if not self.ws_clients:
            return
        dead = set()
        for ws in list(self.ws_clients):
            try:
                # Feedback ya log string'i ya JSON olabilir
                try:
                    parsed = json.loads(text)
                    await ws.send(json.dumps(parsed))
                except Exception:
                    await ws.send(json.dumps({'log': text}))
            except Exception:
                dead.add(ws)
        self.ws_clients -= dead


bridge_node = None


async def ws_handler(websocket):
    """Her WebSocket bağlantısı için çalışır."""
    global bridge_node
    addr = websocket.remote_address
    print(f"[BRIDGE] İstemci bağlandı: {addr}")
    bridge_node.ws_clients.add(websocket)

    try:
        async for raw in websocket:
            try:
                msg = json.loads(raw)
            except Exception:
                continue

            bridge_node.publish_cmd(msg)
            mt = msg.get('type', '')
            if mt and mt not in ('control',):
                # Kısa log
                summary = {k: v for k, v in msg.items() if k != 'ts'}
                print(f"[BRIDGE] → {json.dumps(summary, ensure_ascii=False)}")

    except websockets.exceptions.ConnectionClosed:
        pass
    except Exception as e:
        print(f"[BRIDGE] handler hatası: {e}")
    finally:
        bridge_node.ws_clients.discard(websocket)
        print(f"[BRIDGE] İstemci koptu: {addr}")


async def main_ws():
    bridge_node.ws_loop = asyncio.get_running_loop()
    async with websockets.serve(ws_handler, WS_HOST, WS_PORT):
        print(f"[BRIDGE] WebSocket server: ws://{WS_HOST}:{WS_PORT}")
        await asyncio.Future()


def ros_spin_thread(node):
    rclpy.spin(node)


def main():
    global bridge_node
    rclpy.init()
    bridge_node = BridgeNode()

    t = threading.Thread(target=ros_spin_thread, args=(bridge_node,), daemon=True)
    t.start()

    try:
        asyncio.run(main_ws())
    except KeyboardInterrupt:
        print("\n[BRIDGE] Kapatılıyor...")
    finally:
        try:
            bridge_node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
