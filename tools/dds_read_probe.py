"""DDS 诊断：直接轮询 SDK reader，隔离回调线程与数据链路问题。"""
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "line_demo"))
from dds_network import initialize
from cyclonedds.domain import DomainParticipant
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

domain = int(sys.argv[1])
initialize("lo", domain)
participant = DomainParticipant(domain)
reader = DataReader(participant, Topic(participant, "rt/lowstate", LowState_))
deadline = time.monotonic() + 5
while time.monotonic() < deadline:
    samples = reader.take(10)
    if samples:
        print(f"POLLED_STATE tick={samples[-1].tick}", flush=True)
        break
    time.sleep(.01)
else:
    raise SystemExit("原生 reader 也未收到 LowState")
