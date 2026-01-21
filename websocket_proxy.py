#!/usr/bin/env python3
"""
WebSocket Proxy Server

This script creates a WebSocket server that acts as a proxy between
the drone simulator running in a browser and external client applications.

It forwards messages from clients to the browser and vice versa.
"""

import asyncio
import websockets
import json
import logging
import argparse
import signal
import sys

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
logger = logging.getLogger('websocket_proxy')

class WebSocketProxy:
    def __init__(self, host='localhost', port=8080):
        self.host = host
        self.port = port
        self.browser_connection = None
        self.client_connections = set()
        self.running = False
        self.server = None
    
    async def start(self):
        """Start the WebSocket proxy server"""
        self.running = True
        
        # Setup signal handlers for graceful shutdown (Unix only)
        if sys.platform != 'win32':
            loop = asyncio.get_running_loop()
            for sig in (signal.SIGINT, signal.SIGTERM):
                loop.add_signal_handler(sig, lambda: asyncio.create_task(self.shutdown()))
        
        try:
            self.server = await websockets.serve(
                self.handle_connection,
                self.host,
                self.port
            )
            logger.info(f"WebSocket Proxy Server started on ws://{self.host}:{self.port}")
            await self.server.wait_closed()
        except Exception as e:
            logger.error(f"Error starting WebSocket server: {e}")
            self.running = False
    
    async def handle_connection(self, websocket):
        """Handle new WebSocket connections"""
        # If browser is not connected yet, this connection is browser
        if self.browser_connection is None:
            await self.handle_browser_connection(websocket)
        else:
            await self.handle_client_connection(websocket)

    
    async def handle_browser_connection(self, websocket):
        """Handle connection from the browser simulator"""
        self.browser_connection = websocket
        logger.info("Browser connected")
        
        try:
            # Listen for messages from the browser
            async for message in websocket:
                # Forward message to all clients
                await self.broadcast_to_clients(message)
        except websockets.exceptions.ConnectionClosed:
            logger.info("Browser disconnected")
        finally:
            # Clean up browser connection
            self.browser_connection = None
            # Notify all clients that browser is disconnected
            await self.broadcast_to_clients(json.dumps({
                "type": "status",
                "data": {
                    "connected": False,
                    "message": "Browser disconnected"
                }
            }))
    
    async def handle_client_connection(self, websocket):
        """Handle connection from an external client"""
        # Add to client connections set
        self.client_connections.add(websocket)
        client_id = id(websocket)
        logger.info(f"Client {client_id} connected. Total clients: {len(self.client_connections)}")
        
        # Send current browser connection status
        await websocket.send(json.dumps({
            "type": "status",
            "data": {
                "connected": self.browser_connection is not None,
                "message": "Connected to proxy server"
            }
        }))
        
        try:
            # Listen for messages from the client
            async for message in websocket:
                # Forward message to the browser if connected
                if self.browser_connection:
                    await self.browser_connection.send(message)
                else:
                    # Inform client that browser is not connected
                    await websocket.send(json.dumps({
                        "type": "error",
                        "data": {
                            "message": "Browser not connected. Cannot forward message."
                        }
                    }))
        except websockets.exceptions.ConnectionClosed:
            logger.info(f"Client {client_id} disconnected")
        finally:
            # Remove from client connections (use discard to avoid KeyError if already removed)
            self.client_connections.discard(websocket)
            logger.info(f"Client {client_id} removed. Total clients: {len(self.client_connections)}")
    
    async def broadcast_to_clients(self, message):
        """Send message to all connected clients"""
        if not self.client_connections:
            return

        # Send message to all clients (iterate over a copy to avoid modification during iteration)
        disconnected_clients = set()
        for client in list(self.client_connections):
            try:
                await client.send(message)
            except websockets.exceptions.ConnectionClosed:
                disconnected_clients.add(client)

        # Clean up disconnected clients
        self.client_connections -= disconnected_clients
    
    async def shutdown(self):
        """Gracefully shut down the server"""
        if not self.running:
            return
        
        logger.info("Shutting down server...")
        self.running = False
        
        # Close all client connections
        close_tasks = []
        for client in list(self.client_connections):
            close_tasks.append(client.close())
        
        # Close browser connection
        if self.browser_connection:
            close_tasks.append(self.browser_connection.close())
        
        # Wait for all connections to close
        if close_tasks:
            await asyncio.gather(*close_tasks, return_exceptions=True)
        
        # Close the server
        if self.server:
            self.server.close()
            await self.server.wait_closed()
        
        logger.info("Server shutdown complete")

def main():
    parser = argparse.ArgumentParser(description="WebSocket Proxy Server")
    parser.add_argument("--host", default="localhost", help="Host to bind the server to")
    parser.add_argument("--port", type=int, default=8080, help="Port to listen on")
    parser.add_argument("--log-level", default="INFO", choices=["DEBUG", "INFO", "WARNING", "ERROR"], 
                        help="Log level")
    
    args = parser.parse_args()
    
    # Set log level
    logger.setLevel(getattr(logging, args.log_level))
    
    # Start proxy server
    proxy = WebSocketProxy(args.host, args.port)
    
    try:
        asyncio.run(proxy.start())
    except KeyboardInterrupt:
        logger.info("Server stopped by user")
   
    return 0

if __name__ == "__main__":
    print('\033]0;WebSocket Proxy\007', end='', flush=True)
    sys.exit(main()) 
