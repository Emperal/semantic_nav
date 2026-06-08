#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import json
import socket
import sys

from bsonrpc import JSONRpc


def call_rpc(host, port, command, args):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(float(args.timeout))
    s.connect((host, port))

    rpc = JSONRpc(s)
    server = rpc.get_peer_proxy()

    try:
        if command == "reload":
            result = server.reload_map()

        elif command == "list":
            result = server.list_objects()

        elif command == "labels":
            result = server.list_labels()

        elif command == "state":
            result = server.get_state()

        elif command == "nav_state":
            result = server.get_nav_state()

        elif command == "get_object":
            result = server.get_object(int(args.object_id))

        elif command == "find_class":
            result = server.find_by_class(int(args.class_id))

        elif command == "find_label":
            result = server.find_by_label(str(args.label))

        elif command == "goto_object":
            result = server.goto_object(
                int(args.object_id),
                float(args.approach_distance),
            )

        elif command == "goto_class":
            result = server.goto_class(
                int(args.class_id),
                float(args.approach_distance),
            )

        elif command == "goto_label":
            result = server.goto_label(
                str(args.label),
                float(args.approach_distance),
            )

        else:
            raise ValueError(f"unknown command: {command}")

    finally:
        rpc.close()

    return result


def print_result(result):
    print(json.dumps(result, ensure_ascii=False, indent=2))


def main():
    parser = argparse.ArgumentParser(
        description="Client for semantic object RPC server."
    )

    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=6001)
    parser.add_argument("--timeout", type=float, default=5.0)

    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("reload", help="Reload semantic npz map")
    sub.add_parser("list", help="List all semantic objects")
    sub.add_parser("labels", help="List available semantic labels")
    sub.add_parser("state", help="Get semantic server state")
    sub.add_parser("nav_state", help="Get navigation RPC server state")

    p = sub.add_parser("get_object", help="Get one object by object_id")
    p.add_argument("object_id", type=int)

    p = sub.add_parser("find_class", help="Find objects by class_id")
    p.add_argument("class_id", type=int)

    p = sub.add_parser("find_label", help="Find objects by semantic label")
    p.add_argument("label", type=str)

    p = sub.add_parser("goto_object", help="Navigate to one object by object_id")
    p.add_argument("object_id", type=int)
    p.add_argument("--approach-distance", type=float, default=0.8)

    p = sub.add_parser("goto_class", help="Navigate to best object with given class_id")
    p.add_argument("class_id", type=int)
    p.add_argument("--approach-distance", type=float, default=0.8)

    p = sub.add_parser("goto_label", help="Navigate to best object with given label")
    p.add_argument("label", type=str)
    p.add_argument("--approach-distance", type=float, default=0.8)

    args = parser.parse_args()

    try:
        result = call_rpc(args.host, args.port, args.command, args)
        print_result(result)

    except ConnectionRefusedError:
        print(
            json.dumps(
                {
                    "ok": False,
                    "error": "connection refused",
                    "message": f"Cannot connect to semantic RPC server at {args.host}:{args.port}. Is semantic_object_rpc_server running?",
                },
                ensure_ascii=False,
                indent=2,
            )
        )
        sys.exit(2)

    except socket.timeout:
        print(
            json.dumps(
                {
                    "ok": False,
                    "error": "socket timeout",
                    "message": f"Timeout connecting to semantic RPC server at {args.host}:{args.port}.",
                },
                ensure_ascii=False,
                indent=2,
            )
        )
        sys.exit(2)

    except Exception as e:
        print(
            json.dumps(
                {
                    "ok": False,
                    "error": type(e).__name__,
                    "message": str(e),
                },
                ensure_ascii=False,
                indent=2,
            )
        )
        sys.exit(1)


if __name__ == "__main__":
    main()