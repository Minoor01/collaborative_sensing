from pathlib import Path
from rosbags.highlevel import AnyReader


def inspect_bag(filename):
    print(f"\n===== {filename} =====")

    with AnyReader([Path(filename)]) as reader:
        for connection in reader.connections:
            print(
                connection.topic,
                "|",
                connection.msgtype
            )


inspect_bag(r"bags\sample-ground.bag")
inspect_bag(r"bags\sample-aerial.bag")
