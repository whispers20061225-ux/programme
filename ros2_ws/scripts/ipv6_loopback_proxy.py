import asyncio
import contextlib
import socket
import sys


LISTEN_HOST = "::1"
LISTEN_PORT = 8765
TARGET_HOST = "127.0.0.1"
TARGET_PORT = 8765


async def _pipe(reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
    try:
        while True:
            chunk = await reader.read(65536)
            if not chunk:
                break
            writer.write(chunk)
            await writer.drain()
    finally:
        with contextlib.suppress(Exception):
            writer.close()
            await writer.wait_closed()


async def _handle_client(
    client_reader: asyncio.StreamReader,
    client_writer: asyncio.StreamWriter,
) -> None:
    try:
        upstream_reader, upstream_writer = await asyncio.open_connection(
            TARGET_HOST,
            TARGET_PORT,
        )
    except Exception:
        with contextlib.suppress(Exception):
            client_writer.close()
            await client_writer.wait_closed()
        return

    try:
        await asyncio.gather(
            _pipe(client_reader, upstream_writer),
            _pipe(upstream_reader, client_writer),
        )
    finally:
        with contextlib.suppress(Exception):
            upstream_writer.close()
            await upstream_writer.wait_closed()
        with contextlib.suppress(Exception):
            client_writer.close()
            await client_writer.wait_closed()


async def main() -> None:
    server = await asyncio.start_server(
        _handle_client,
        LISTEN_HOST,
        LISTEN_PORT,
        family=socket.AF_INET6,
    )
    addrs = ", ".join(str(sock.getsockname()) for sock in (server.sockets or []))
    print(f"ipv6 loopback proxy listening on {addrs} -> {TARGET_HOST}:{TARGET_PORT}")
    async with server:
        await server.serve_forever()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        sys.exit(0)
